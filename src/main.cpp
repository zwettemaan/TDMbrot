#include <Arduino.h>
#include <TFT_eSPI.h>
#include <esp_task_wdt.h>

// Rendering mode: set to 1 to render while calculating, 0 to calculate then render
#define RENDER_WHILE_CALCULATING 1

// Algorithm mode: set to 1 for optimized recursive algorithm, 0 for naive nested loop
#define USE_OPTIMIZED_ALGORITHM 1

// Button pins
#define BUTTON_1 0   // Left button (also BOOT button)
#define BUTTON_2 14  // Right button

TFT_eSPI tft = TFT_eSPI();

// Display dimensions in landscape mode
#define WIDTH 320
#define HEIGHT 170

// ============================================================================
// Fixed-point arithmetic (8.24 format) - 8 integer bits, 24 fractional bits
// ============================================================================
#define FP_SHIFT 24
#define FP_SCALE 16777216  // 2^24

// Type aliases for fixed-point numbers
typedef int32_t TFixedPoint;      // Standard fixed-point (8.24)
typedef int64_t TWideFixedPoint;  // Wide fixed-point for intermediate results

// Grid to store Mandelbrot iteration counts
uint16_t grid[WIDTH][HEIGHT];

// Max steps for color mapping (set during renderMandelbrot)
int g_curMaxSteps = 100;

// Current view parameters
float g_xm = -0.770;
float g_ym = 0.095;
float g_dm = 0.01;
bool g_calculating = false;
const int c_maxSteps = 1000;

// Multi-core work queue system
SemaphoreHandle_t g_renderMutex = NULL;
QueueHandle_t g_workQueue = NULL;

volatile int g_activeWorkers = 0;

volatile int g_busyWorkers = 0;

// Work item structure for the queue
struct WorkItem {
  int xi;           // Grid start X
  TFixedPoint x;    // World X coordinate (fixed-point)
  int nx;           // Number of X pixels
  int yi;           // Grid start Y
  TFixedPoint y;    // World Y coordinate (fixed-point)
  int ny;           // Number of Y pixels
  TFixedPoint d;    // World scale (fixed-point)
  int maxSteps;     // Max iterations
};

float g_xTop;
float g_yTop;
float g_d_Inv;

// Global rendering parameters
float g_renderScale = 0.0;
int g_renderMaxSteps = 0;

uint16_t stepsToColor(int steps, int maxSteps);
void processWorkItem(WorkItem& item);

// Convert float to fixed-point
inline TFixedPoint toFixed(float value) {
  return (TFixedPoint)(value * FP_SCALE);
}

// Convert float constant to fixed-point at compile time
#define FP_CONST(x) ((TFixedPoint)((x) * FP_SCALE))

// Multiply two fixed-point numbers (returns wide fixed-point)
inline TWideFixedPoint fpMul(TFixedPoint a, TFixedPoint b) {
  return ((TWideFixedPoint)a * b) >> FP_SHIFT;
}

// Multiply two fixed-point numbers with double shift (for 2*x*y operations)
inline TWideFixedPoint fpMul2(TFixedPoint a, TFixedPoint b) {
  return ((TWideFixedPoint)a * b) >> (FP_SHIFT - 1);
}

// Square a fixed-point number (returns wide fixed-point)
inline TWideFixedPoint fpSquare(TFixedPoint value) {
  return fpMul(value, value);
}

// Add two fixed-point numbers
inline TFixedPoint fpAdd(TFixedPoint a, TFixedPoint b) {
  return a + b;
}

// Add two wide fixed-point numbers
inline TWideFixedPoint fpAdd(TWideFixedPoint a, TWideFixedPoint b) {
  return a + b;
}

// Add fixed-point to wide fixed-point
inline TWideFixedPoint fpAdd(TWideFixedPoint a, TFixedPoint b) {
  return a + b;
}

// Subtract two fixed-point numbers
inline TFixedPoint fpSub(TFixedPoint a, TFixedPoint b) {
  return a - b;
}

// Subtract two wide fixed-point numbers
inline TWideFixedPoint fpSub(TWideFixedPoint a, TWideFixedPoint b) {
  return a - b;
}

// Check if a point is inside the Mandelbrot set (main cardioid or period-2 bulb)
// Returns true if the point is definitely inside the set
inline bool isInsideOfSet(TFixedPoint px, TFixedPoint py) {
  // Calculate squares once
  const TWideFixedPoint px2 = fpSquare(px);
  const TWideFixedPoint py2 = fpSquare(py);

  // Quick check: main cardioid
  // Formula: q = (x-0.25)^2 + y^2, check if q*(q + (x-0.25)) < 0.25*y^2
  // Expanded: q = (x-0.25)^2 + y^2 = x^2 - 0.5*x + 0.0625 + y^2
  // Reuse calculated x^2 and y^2 to avoid redundant computation
  const TWideFixedPoint q_x = px - FP_CONST(0.25);
  const TWideFixedPoint q = px2 + py2 - (px >> 1) + FP_CONST(0.0625);  // px>>1 divides by 2 (i.e., 0.5*x)

  // Check: q * (q + (x-0.25)) < 0.25 * y^2
  const TWideFixedPoint cardioid_lhs = fpMul(q, fpAdd(q, q_x));
  const TWideFixedPoint cardioid_rhs = py2 >> 2;  // Divide by 4 = multiply by 0.25
  if (cardioid_lhs < cardioid_rhs) {
    return true;
  }

  // Check period-2 bulb: (x + 1)^2 + y^2 < 0.0625
  // Expand: (x + 1)^2 + y^2 = x^2 + 2*x + 1 + y^2
  // Reuse pre-calculated squares
  const TWideFixedPoint bulb_dist = px2 + py2 + (px << 1) + FP_SCALE;  // px<<1 is 2*x
  if (bulb_dist < FP_CONST(0.0625)) {
    return true;
  }

  return false;
}

void reset() {
  // Clear screen
  tft.fillScreen(TFT_BLACK);
  memset(grid, 0, sizeof(grid));
}

// Calculate number of steps for Mandelbrot iteration
int nSteps(TFixedPoint cx, TFixedPoint cy, int maxSteps) {
  // Quick check on initial point (c value only, not iterated z)
  if (isInsideOfSet(cx, cy)) {
    return maxSteps;
  }

  // Mandelbrot iteration
  TFixedPoint x = 0;
  TFixedPoint y = 0;
  int steps = 0;

  while (steps < maxSteps) {
    // Calculate squares (safe to cast since we check for escape at 4)
    const TFixedPoint x2 = (TFixedPoint)fpSquare(x);
    const TFixedPoint y2 = (TFixedPoint)fpSquare(y);

    // Check if we've escaped (x^2 + y^2 > 4)
    if (fpAdd(x2, y2) > FP_CONST(4.0)) {
      break;
    }

    // Mandelbrot iteration: z = z^2 + c
    const TFixedPoint xtemp = fpAdd(fpSub(x2, y2), cx);
    y = (TFixedPoint)fpAdd(fpMul2(x, y), cy);
    x = xtemp;

    steps++;
  }

  return steps;
}

// Per-core pending region buffers to reduce mutex contention
#define MAX_PENDING_REGIONS 100
struct PendingRegion {
  int x;   // Start X
  int y;   // Start Y
  int nx;  // Width
  int ny;  // Height
};
PendingRegion g_pendingRegions[2][MAX_PENDING_REGIONS]; // [coreId][regions]
int g_pendingCount[2] = {0, 0};
__thread int g_coreId = -1; // Thread-local core ID

// Flush pending regions to display (must hold mutex)
void flushPendingRegions(const int coreId) {
  int* pc = &g_pendingCount[coreId];
  for (int i = 0; i < *pc; i++) {
    PendingRegion& pr = g_pendingRegions[coreId][i];
    // Render the region from grid data
    for (int xi = 0; xi < pr.nx; xi++) {
      for (int yi = 0; yi < pr.ny; yi++) {
        const uint16_t color = stepsToColor(grid[pr.x + xi][pr.y + yi], g_curMaxSteps);
        tft.drawPixel(pr.x + xi, pr.y + yi, color);
      }
    }
  }
  *pc = 0;
}

// Set grid value (no rendering)
inline void setGridPixel(const int xi, const int yi, const uint16_t value) {
  grid[xi][yi] = value;
}

// Render a region from grid to display (with buffering if mutex busy)
void renderRegion(const int xi, const int yi, const int nx, const int ny) {
#if RENDER_WHILE_CALCULATING
  if (g_coreId < 0) return; // Not in worker context

  bool hasSemaphore = false;
  // Check if buffer will be full AFTER adding this region
  if (g_pendingCount[g_coreId] >= MAX_PENDING_REGIONS) {
    // Buffer full - must wait for mutex then flush first
    xSemaphoreTake(g_renderMutex, portMAX_DELAY);
    hasSemaphore = true;
    flushPendingRegions(g_coreId);
  }

  // Add region to buffer
  PendingRegion& pr = g_pendingRegions[g_coreId][g_pendingCount[g_coreId]];
  pr.x = xi;
  pr.y = yi;
  pr.nx = nx;
  pr.ny = ny;
  g_pendingCount[g_coreId]++;

  // Try to get mutex without blocking (if we don't already have it)
  if (hasSemaphore || xSemaphoreTake(g_renderMutex, 0) == pdTRUE) {
    // Got mutex - flush all pending regions (including the one we just added)
    flushPendingRegions(g_coreId);
    xSemaphoreGive(g_renderMutex);
  }
  // else: buffer has space, keep calculating
#endif
}

// Map iteration count to color
uint16_t stepsToColor(const int steps, const int maxSteps) {

  if (steps >= maxSteps) {
    return TFT_BLACK; // Inside the set
  }

  // Use logarithmic scale to spread out low iteration counts
  // Most detail is at the edges with low iteration counts
  const float normalized = log(steps + 1) / log(maxSteps + 1);
  uint8_t hue = (uint8_t)(normalized * 255);
  uint8_t r, g, b;

  // Simple RGB mapping
  if (hue < 85) {
    r = hue * 3;
    g = 255 - hue * 3;
    b = 0;
  } else if (hue < 170) {
    hue -= 85;
    r = 255 - hue * 3;
    g = 0;
    b = hue * 3;
  } else {
    hue -= 170;
    r = 0;
    g = hue * 3;
    b = 255 - hue * 3;
  }

  return tft.color565(r, g, b);
}

// Helper to add work item to queue or process directly if queue is full
void enqueueOrProcess(const int xi, const TFixedPoint x, const int nx, const int yi, const TFixedPoint y, const int ny, const TFixedPoint d, const int maxSteps) {
  WorkItem item = {xi, x, nx, yi, y, ny, d, maxSteps};

  // If queue has space, enqueue
  if (xQueueSend(g_workQueue, &item, 0) == pdTRUE) {
    return;
  }

  // Queue is full - process directly
  processWorkItem(item);
}

// Process a single work item - either subdivide or calculate
void processWorkItem(WorkItem& item) {

  const int nx = item.nx;
  const int ny = item.ny;

  if (nx < 1 || ny < 1) {
    return;
  }

  const int xi = item.xi;
  const int yi = item.yi;
  const TFixedPoint x = item.x;
  const TFixedPoint y = item.y;
  const int maxSteps = item.maxSteps;

  if (nx == 1 && ny == 1) {
    const int n = nSteps(x, y, maxSteps);
    setGridPixel(xi, yi, n);
    renderRegion(xi, yi, 1, 1);
    return;
  }

  const TFixedPoint d = item.d;

  if (nx == 1) {
    const int n1 = nSteps(x, y, maxSteps);
    const int n2 = nSteps(x, y + d * (ny - 1), maxSteps);

    setGridPixel(xi, yi, n1);
    setGridPixel(xi, yi + ny - 1, n2);
    renderRegion(xi, yi, 1, 1);
    renderRegion(xi, yi + ny - 1, 1, 1);

    if (ny == 2) {
      return;
    }

    const int ny1 = (ny-2)/2;
    const int ny2 = ny-2-ny1;

    enqueueOrProcess(xi, x, 1, yi + 1, y + d, ny1, d, maxSteps);
    enqueueOrProcess(xi, x, 1, yi + 1 + ny1, y + d * (1 + ny1), ny2, d, maxSteps);
    return;
  }

  if (ny == 1) {
    const int n1 = nSteps(x, y, maxSteps);
    const int n2 = nSteps(x + d * (nx - 1), y, maxSteps);

    setGridPixel(xi, yi, n1);
    setGridPixel(xi + nx - 1, yi, n2);
    renderRegion(xi, yi, 1, 1);
    renderRegion(xi + nx - 1, yi, 1, 1);

    if (nx == 2) {
      return;
    }

    const int nx1 = (nx-2)/2;
    const int nx2 = nx-2-nx1;

    enqueueOrProcess(xi + 1, x + d, nx1, yi, y, 1, d, maxSteps);
    enqueueOrProcess(xi + 1 + nx1, x + d * (1 + nx1), nx2, yi, y, 1, d, maxSteps);
    return;
  }

  int n1 = nSteps(x, y, maxSteps);
  int n2 = nSteps(x + d * (nx - 1), y, maxSteps);
  int n3 = nSteps(x, y + d * (ny - 1), maxSteps);
  int n4 = nSteps(x + d * (nx - 1), y + d * (ny - 1), maxSteps);

  if (n1 == n2 && n1 == n3 && n1 == n4 && (n1 < maxSteps)) {
    // Fill entire block - set grid values then render as single region
    for (int xi2 = 0; xi2 < nx; xi2++) {
      for (int yi2 = 0; yi2 < ny; yi2++) {
        setGridPixel(xi + xi2, yi + yi2, n1);
      }
    }
    asm volatile("" ::: "memory"); // Compiler barrier - ensure grid writes complete before render
    renderRegion(xi, yi, nx, ny);
    return;
  }

  setGridPixel(xi, yi, n1);
  setGridPixel(xi + nx - 1, yi, n2);
  setGridPixel(xi, yi + ny - 1, n3);
  setGridPixel(xi + nx - 1, yi + ny - 1, n4);

  // Render corners as individual pixels (not contiguous regions)
  renderRegion(xi, yi, 1, 1);
  renderRegion(xi + nx - 1, yi, 1, 1);
  renderRegion(xi, yi + ny - 1, 1, 1);
  renderRegion(xi + nx - 1, yi + ny - 1, 1, 1);

  // Subdivide and add to queue (or process directly if queue is full)
  enqueueOrProcess(xi + 1, x + d, nx-2, yi, y, 1, d, maxSteps);
  enqueueOrProcess(xi + 1, x + d, nx-2, yi + ny - 1, y + d * (ny - 1), 1, d, maxSteps);

  enqueueOrProcess(xi, x, 1, yi + 1, y + d, ny-2, d, maxSteps);
  enqueueOrProcess(xi + nx - 1, x + d * (nx - 1), 1, yi + 1, y + d, ny-2, d, maxSteps);

  const int nx1 = (nx-2)/2;
  const int nx2 = nx-2-nx1;

  const int ny1 = (ny-2)/2;
  const int ny2 = ny-2-ny1;
  enqueueOrProcess(xi + 1,       x + d,           nx1, yi + 1,       y + d,         ny1, d, maxSteps);
  enqueueOrProcess(xi + 1,       x + d,           nx1, yi + 1 + ny1, y + d + ny1 * d, ny2, d, maxSteps);
  enqueueOrProcess(xi + 1 + nx1, x + d + nx1 * d,   nx2, yi + 1,       y + d,         ny1, d, maxSteps);
  enqueueOrProcess(xi + 1 + nx1, x + d + nx1 * d,   nx2, yi + 1 + ny1, y + d + ny1 * d, ny2, d, maxSteps);
}

// Worker task - pulls work from queue and processes it
void workerTask(void* params) {

  const int coreId = (int)params;
  g_coreId = coreId; // Set thread-local core ID

  bool working = true;

  while (working) {
    WorkItem item;

    // Try to get work from queue (wait up to 1ms to allow watchdog resets)
    if (xQueueReceive(g_workQueue, &item, 0) == pdTRUE) {
      const int flag = 1 << g_coreId;
      __atomic_or_fetch(&g_busyWorkers, flag, __ATOMIC_SEQ_CST);

      // Process the work item
#if USE_OPTIMIZED_ALGORITHM
      processWorkItem(item);
#else
      // Naive algorithm - just calculate the region
      TFixedPoint scale = item.d;
      for (int xi = item.xi; xi < item.xi + item.nx; xi++) {
        for (int yi = item.yi; yi < item.yi + item.ny; yi++) {
          TFixedPoint x = item.x + (xi - item.xi) * scale;
          TFixedPoint y = item.y + (yi - item.yi) * scale;
          int n = nSteps(x, y, item.maxSteps);
          setGridPixel(xi, yi, n);
        }
      }
#endif

        __atomic_and_fetch(&g_busyWorkers, ~flag, __ATOMIC_SEQ_CST);
    } else {
      if (g_busyWorkers == 0 && uxQueueMessagesWaiting(g_workQueue) == 0) {
        working = false;
#if RENDER_WHILE_CALCULATING
        // Flush any remaining pending regions before exiting
        if (xSemaphoreTake(g_renderMutex, portMAX_DELAY) == pdTRUE) {
          flushPendingRegions(coreId);
          xSemaphoreGive(g_renderMutex);
        }
#endif
      }
    }
  }
  __atomic_sub_fetch(&g_activeWorkers, 1, __ATOMIC_SEQ_CST);
  vTaskDelete(NULL);
}

// Calculate and render Mandelbrot set
// xm, ym = top-left corner of window
// dm = width of window (height = dm * 17/32)
void renderMandelbrot(const float xm, const float ym, const float dm, const int maxSteps) {

  reset();

  g_calculating = true;

  g_xTop = xm;
  g_yTop = ym;
  g_d_Inv = 1.0 / dm;

  unsigned long startTime = millis();
  g_curMaxSteps = maxSteps;

  float window_height = dm * 17.0 / 32.0;
  Serial.printf("Calculating Mandelbrot: top-left=(%.4f, %.4f), width=%.4f, height=%.4f\n", xm, ym, dm, window_height);
  Serial.printf("Render mode: %s\n", RENDER_WHILE_CALCULATING ? "while-calculating" : "post-render");
  Serial.printf("Algorithm: %s\n", USE_OPTIMIZED_ALGORITHM ? "optimized-recursive" : "naive-loop");
  Serial.println("Using queue-based dual-core rendering");

  // Clear and reset work queue
  if (g_workQueue != NULL) {
    xQueueReset(g_workQueue);
  }

  g_activeWorkers = 0;
  g_busyWorkers = 0;
  g_renderScale = dm / 320.0;
  g_renderMaxSteps = maxSteps;

  // Reset pending region buffers
  g_pendingCount[0] = 0;
  g_pendingCount[1] = 0;

  __atomic_add_fetch(&g_activeWorkers, 1, __ATOMIC_SEQ_CST);
  // Create worker tasks on both cores (priority 0 same as IDLE)
  xTaskCreatePinnedToCore(
    workerTask,          // Task function
    "Worker0",           // Task name
    8192,                // Stack size
    (void*)0,            // Core ID as parameter
    0,                   // Priority
    NULL,                // Task handle
    0                    // Core 0
  );

  __atomic_add_fetch(&g_activeWorkers, 1, __ATOMIC_SEQ_CST);
  xTaskCreatePinnedToCore(
    workerTask,          // Task function
    "Worker1",           // Task name
    8192,                // Stack size
    (void*)1,            // Core ID as parameter
    0,                   // Priority
    NULL,                // Task handle
    1                    // Core 1
  );

  // Add initial work item to queue
#if USE_OPTIMIZED_ALGORITHM
  WorkItem initialWorkItem;
  initialWorkItem.xi = 0;
  initialWorkItem.x = toFixed(xm);
  initialWorkItem.nx = WIDTH;
  initialWorkItem.yi = 0;
  initialWorkItem.y = toFixed(ym);
  initialWorkItem.ny = HEIGHT;
  initialWorkItem.d = toFixed(g_renderScale);
  initialWorkItem.maxSteps = maxSteps;
  xQueueSend(g_workQueue, &initialWorkItem, 0);
#else
  // For naive algorithm, split into chunks for better distribution
  int chunkSize = 40; // Process in 40-pixel wide vertical strips
  TFixedPoint xm_fp = toFixed(xm);
  TFixedPoint ym_fp = toFixed(ym);
  TFixedPoint scale_fp = toFixed(g_renderScale);
  for (int xi = 0; xi < WIDTH; xi += chunkSize) {
    int nx = min(chunkSize, WIDTH - xi);
    WorkItem item;
    item.xi = xi;
    item.x = xm_fp + xi * scale_fp;
    item.nx = nx;
    item.yi = 0;
    item.y = ym_fp;
    item.ny = HEIGHT;
    item.d = scale_fp;
    item.maxSteps = maxSteps;
    xQueueSend(g_workQueue, &item, 0);
  }
#endif

  // Wait for all workers to finish
  while (g_activeWorkers) {
    delay(10);
  }

  unsigned long calcTime = millis();
  Serial.printf("Calculation complete: %lu ms\n", calcTime - startTime);

#if !RENDER_WHILE_CALCULATING
  // Render to display (post-render mode)
  for (int xi = 0; xi < WIDTH; xi++) {
    for (int yi = 0; yi < HEIGHT; yi++) {
      uint16_t color = stepsToColor(grid[xi][yi], maxSteps);
      tft.drawPixel(xi, yi, color);
    }
  }

  unsigned long renderTime = millis();
  Serial.printf("Render complete: %lu ms\n", renderTime - calcTime);
  Serial.printf("Total time: %lu ms\n", renderTime - startTime);
#else
  Serial.printf("Total time: %lu ms\n", calcTime - startTime);
#endif

  g_calculating = false;
}

// Button handling
unsigned long button1PressTime = 0;
unsigned long button2PressTime = 0;
bool button1WasPressed = false;
bool button2WasPressed = false;
unsigned long button1ReleaseTime = 0;
unsigned long button2ReleaseTime = 0;
bool button1PendingSingle = false;
bool button2PendingSingle = false;
bool button1DoubleInProgress = false;
bool button2DoubleInProgress = false;

#define LONG_PRESS_TIME 500
#define DOUBLE_PRESS_TIME 300

void handleButtons() {
  if (g_calculating) return; // Ignore buttons while calculating

  bool btn1 = digitalRead(BUTTON_1) == LOW;
  bool btn2 = digitalRead(BUTTON_2) == LOW;

  // Check for pending single presses that have timed out
  if (button1PendingSingle && (millis() - button1ReleaseTime >= DOUBLE_PRESS_TIME)) {
    Serial.println("Button 1 single press: Shift left");
    g_xm -= g_dm / 2.0;
    renderMandelbrot(g_xm, g_ym, g_dm, c_maxSteps);
    button1PendingSingle = false;
    button1ReleaseTime = 0;
  }

  if (button2PendingSingle && (millis() - button2ReleaseTime >= DOUBLE_PRESS_TIME)) {
    Serial.println("Button 2 single press: Shift right");
    g_xm += g_dm / 2.0;
    renderMandelbrot(g_xm, g_ym, g_dm, c_maxSteps);
    button2PendingSingle = false;
    button2ReleaseTime = 0;
  }

  // Button 1 handling
  if (btn1 && !button1WasPressed) {
    button1PressTime = millis();
    button1WasPressed = true;

    // Check if this is a double press
    if (button1PendingSingle && (millis() - button1ReleaseTime < DOUBLE_PRESS_TIME)) {
      // Double press detected - set flag for when button is released
      button1DoubleInProgress = true;
      button1PendingSingle = false;
      button1ReleaseTime = 0;
    }
  } else if (!btn1 && button1WasPressed) {
    unsigned long pressDuration = millis() - button1PressTime;
    button1WasPressed = false;

    if (pressDuration >= LONG_PRESS_TIME) {
      // Long press: zoom in
      Serial.println("Button 1 long press: Zoom in");
      button1PendingSingle = false;
      button1DoubleInProgress = false;
      button1ReleaseTime = 0;
      g_dm /= 2.0;
      g_xm += g_dm / 2.0;
      g_ym += (g_dm * 17.0 / 32.0) / 2.0;
      renderMandelbrot(g_xm, g_ym, g_dm, c_maxSteps);
    } else if (button1DoubleInProgress) {
      // This was a double press
      Serial.println("Button 1 double press: Shift up");
      button1DoubleInProgress = false;
      button1PendingSingle = false;
      button1ReleaseTime = 0;
      g_ym -= g_dm / 2.0 * 17.0 / 32.0;
      renderMandelbrot(g_xm, g_ym, g_dm, c_maxSteps);
    } else {
      // Mark as pending single press
      button1PendingSingle = true;
      button1ReleaseTime = millis();
    }
  }

  // Button 2 handling
  if (btn2 && !button2WasPressed) {
    button2PressTime = millis();
    button2WasPressed = true;

    // Check if this is a double press
    if (button2PendingSingle && (millis() - button2ReleaseTime < DOUBLE_PRESS_TIME)) {
      // Double press detected - set flag for when button is released
      button2DoubleInProgress = true;
      button2PendingSingle = false;
      button2ReleaseTime = 0;
    }
  } else if (!btn2 && button2WasPressed) {
    unsigned long pressDuration = millis() - button2PressTime;
    button2WasPressed = false;

    if (pressDuration >= LONG_PRESS_TIME) {
      // Long press: zoom out
      Serial.println("Button 2 long press: Zoom out");
      button2PendingSingle = false;
      button2DoubleInProgress = false;
      button2ReleaseTime = 0;
      float new_dm = g_dm * 2.0;

      // Check if corners would go outside [-2, 2] range
      float new_x_end = g_xm + new_dm;
      float new_y_end = g_ym + new_dm * 17.0 / 32.0;

      // Constrain to keep all corners in [-2, 2]
      if (g_xm >= -2.0 && new_x_end <= 2.0 && g_ym >= -2.0 && new_y_end <= 2.0) {
        g_dm = new_dm;
        g_xm -= g_dm / 4.0;
        g_ym -= (g_dm * 17.0 / 32.0) / 4.0;

        // Clamp to boundaries
        if (g_xm < -2.0) g_xm = -2.0;
        if (g_ym < -2.0) g_ym = -2.0;
        if (g_xm + g_dm > 2.0) g_xm = 2.0 - g_dm;
        if (g_ym + g_dm * 17.0 / 32.0 > 2.0) g_ym = 2.0 - g_dm * 17.0 / 32.0;

        renderMandelbrot(g_xm, g_ym, g_dm, c_maxSteps);
      } else {
        Serial.println("Zoom out limited: would exceed bounds");
      }
    } else if (button2DoubleInProgress) {
      // This was a double press
      Serial.println("Button 2 double press: Shift down");
      button2DoubleInProgress = false;
      button2PendingSingle = false;
      button2ReleaseTime = 0;
      g_ym += g_dm / 2.0 * 17.0 / 32.0;
      renderMandelbrot(g_xm, g_ym, g_dm, c_maxSteps);
    } else {
      // Mark as pending single press
      button2PendingSingle = true;
      button2ReleaseTime = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Create mutex for thread-safe rendering
  g_renderMutex = xSemaphoreCreateMutex();

  // Create work queue (holds up to 200 work items for better distribution)
  g_workQueue = xQueueCreate(200, sizeof(WorkItem));

  // Initialize buttons
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);

  // Initialize the display
  tft.init();
  tft.setRotation(1); // Landscape orientation (320x170)

  // Turn on backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  Serial.printf("TFT dimensions after rotation: %d x %d\n", tft.width(), tft.height());
  Serial.printf("Expected dimensions: %d x %d\n", WIDTH, HEIGHT);

  // Initial render with current global parameters
  renderMandelbrot(g_xm, g_ym, g_dm, c_maxSteps);

  Serial.println("Mandelbrot rendering complete!");
  Serial.println("Button controls:");
  Serial.println("  Button 1 single: Shift left");
  Serial.println("  Button 2 single: Shift right");
  Serial.println("  Button 1 double: Shift up");
  Serial.println("  Button 2 double: Shift down");
  Serial.println("  Button 1 long: Zoom in");
  Serial.println("  Button 2 long: Zoom out");
}

void loop() {
  handleButtons();
  delay(10); // Small delay for button debouncing
}