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

// Display dimensions in portrait mode
#define WIDTH 170
#define HEIGHT 320

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

// Maximum iteration count
const int c_maxIterations = 10000;

// Calculate max steps based on zoom level
int calculateMaxSteps(float dm) {
  // Base iteration count
  const int baseSteps = 200;
  
  // Calculate zoom depth (smaller dm = deeper zoom)
  // Initial dm is around 4.0 (full view), zooming in makes it smaller
  float zoomDepth = log(4.0 / dm) / log(2.0);  // log2 of zoom factor
  
  // Increase steps as we zoom in
  // Add 50 steps per doubling of zoom
  int additionalSteps = (int)(zoomDepth * 50);
  
  // Cap at maximum to avoid very slow renders
  int maxSteps = baseSteps + additionalSteps;
  if (maxSteps > c_maxIterations) maxSteps = c_maxIterations;
  if (maxSteps < baseSteps) maxSteps = baseSteps;
  
  return maxSteps;
}

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
// dm = width of window (height = dm * 320/170)
void renderMandelbrot(const float xm, const float ym, const float dm, const int maxSteps) {

  reset();

  g_calculating = true;

  g_xTop = xm;
  g_yTop = ym;
  g_d_Inv = 1.0 / dm;

  unsigned long startTime = millis();
  g_curMaxSteps = maxSteps;

  float window_height = dm * 320.0 / 170.0;
  Serial.printf("Calculating Mandelbrot: top-left=(%.4f, %.4f), width=%.4f, height=%.4f, maxSteps=%d\n", xm, ym, dm, window_height, maxSteps);
  Serial.printf("Render mode: %s\n", RENDER_WHILE_CALCULATING ? "while-calculating" : "post-render");
  Serial.printf("Algorithm: %s\n", USE_OPTIMIZED_ALGORITHM ? "optimized-recursive" : "naive-loop");
  Serial.println("Using queue-based dual-core rendering");

  // Clear and reset work queue
  if (g_workQueue != NULL) {
    xQueueReset(g_workQueue);
  }

  g_activeWorkers = 0;
  g_busyWorkers = 0;
  g_renderScale = dm / 170.0;
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

// Menu system for button handling
enum MenuState {
  MENU_NONE,      // No menu active
  MENU_DIRECTION, // Direction selection menu
  MENU_ZOOM       // Zoom selection menu
};

MenuState g_menuState = MENU_NONE;
int g_menuSelection = 0; // Current selection index (0-3)

// Direction options: North, East, South, West
const char* g_directions[] = {"N", "E", "S", "W"};
const float g_directionOffsets[][2] = {
  {0.0, -0.5},   // N: shift up
  {0.5, 0.0},    // E: shift right
  {0.0, 0.5},    // S: shift down
  {-0.5, 0.0}    // W: shift left
};

// Zoom options
const float g_zoomFactors[] = {0.5, 0.75, 1.5, 2.0};

// Button state
bool button1WasPressed = false;
bool button2WasPressed = false;

// Draw direction selection menu
void drawDirectionMenu() {
  
  // Draw title text
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 10);
  tft.println("left button = pick");
  tft.setCursor(10, 20);
  tft.println("right button = action");
  
  // Draw directional arrows/labels in compass layout
  const int centerX = WIDTH / 2;
  const int centerY = HEIGHT / 2;
  const int spacing = 40;
  
  // Positions: N, E, S, W
  const int positions[][2] = {
    {centerX, centerY - spacing},      // N
    {centerX + spacing, centerY},      // E
    {centerX, centerY + spacing},      // S
    {centerX - spacing, centerY}       // W
  };
  
  for (int i = 0; i < 4; i++) {
    int x = positions[i][0];
    int y = positions[i][1];
    
    // Draw box around selected item
    if (i == g_menuSelection) {
      tft.fillRect(x - 15, y - 10, 30, 20, TFT_GREEN);
      tft.setTextColor(TFT_BLACK, TFT_GREEN);
    } else {
      tft.fillRect(x - 15, y - 10, 30, 20, TFT_DARKGREY);
      tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
    }
    
    // Draw direction label centered
    tft.setTextSize(2);
    tft.setCursor(x - 6, y - 8);
    tft.print(g_directions[i]);
  }
}

// Draw zoom selection menu
void drawZoomMenu() {
  
  // Draw title text
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 10);
  tft.println("left button = pick");
  tft.setCursor(10, 20);
  tft.println("right button = action");
  
  // Draw zoom options in a grid
  const int startY = 60;
  const int spacing = 35;
  
  for (int i = 0; i < 4; i++) {
    int x = WIDTH / 2 - 30;
    int y = startY + i * spacing;
    
    // Draw box around selected item
    if (i == g_menuSelection) {
      tft.fillRect(x - 5, y - 5, 70, 25, TFT_GREEN);
      tft.setTextColor(TFT_BLACK, TFT_GREEN);
    } else {
      tft.fillRect(x - 5, y - 5, 70, 25, TFT_DARKGREY);
      tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
    }
    
    // Draw zoom factor
    tft.setTextSize(2);
    tft.setCursor(x, y);
    tft.printf("%.2fx", g_zoomFactors[i]);
  }
}

void handleButtons() {
  if (g_calculating) return; // Ignore buttons while calculating

  bool btn1 = digitalRead(BUTTON_1) == LOW;
  bool btn2 = digitalRead(BUTTON_2) == LOW;

  // Button 1 (left) - pressed
  if (btn1 && !button1WasPressed) {
    button1WasPressed = true;
    
    if (g_menuState == MENU_NONE) {
      // First press - open zoom menu
      g_menuState = MENU_ZOOM;
      g_menuSelection = 0;
      drawZoomMenu();
    } else if (g_menuState == MENU_ZOOM) {
      // Cycle through zoom selections
      g_menuSelection = (g_menuSelection + 1) % 4;
      drawZoomMenu();
    } else if (g_menuState == MENU_DIRECTION) {
      // Execute direction shift (opposite button from the one that opened the menu)
      float offsetX = g_directionOffsets[g_menuSelection][0] * g_dm;
      float offsetY = g_directionOffsets[g_menuSelection][1] * g_dm * 320.0 / 170.0;
      
      g_xm += offsetX;
      g_ym += offsetY;
      
      g_menuState = MENU_NONE;
      Serial.printf("Shifting %s\n", g_directions[g_menuSelection]);
      renderMandelbrot(g_xm, g_ym, g_dm, calculateMaxSteps(g_dm));
    }
  } else if (!btn1 && button1WasPressed) {
    button1WasPressed = false;
  }

  // Button 2 (right) - pressed
  if (btn2 && !button2WasPressed) {
    button2WasPressed = true;
    
    if (g_menuState == MENU_NONE) {
      // First press - open direction menu
      g_menuState = MENU_DIRECTION;
      g_menuSelection = 0;
      drawDirectionMenu();
    } else if (g_menuState == MENU_DIRECTION) {
      // Cycle through direction selections
      g_menuSelection = (g_menuSelection + 1) % 4;
      drawDirectionMenu();
    } else if (g_menuState == MENU_ZOOM) {
      // Execute zoom (opposite button from the one that opened the menu)
      float zoomFactor = g_zoomFactors[g_menuSelection];
      float centerX = g_xm + g_dm / 2.0;
      float centerY = g_ym + (g_dm * 320.0 / 170.0) / 2.0;
      
      float new_dm = g_dm / zoomFactor;
      
      // Check if zoom would exceed fixed-point precision
      // With 8.24 format, minimum representable step is 1/2^24 ≈ 6e-8
      // We need at least 2 units per pixel step for safety
      float newScale = new_dm / 170.0;
      float minScale = 2.0 / FP_SCALE;  // Minimum safe scale
      
      if (newScale < minScale) {
        Serial.printf("Zoom limit reached - scale %.2e would be below fixed-point precision %.2e\n", newScale, minScale);
        g_menuState = MENU_NONE;
        renderMandelbrot(g_xm, g_ym, g_dm, calculateMaxSteps(g_dm));
        return;
      }
      
      // Recalculate top-left to keep center stable
      g_dm = new_dm;
      g_xm = centerX - g_dm / 2.0;
      g_ym = centerY - (g_dm * 320.0 / 170.0) / 2.0;
      
      g_menuState = MENU_NONE;
      Serial.printf("Zooming by %.2fx\n", zoomFactor);
      renderMandelbrot(g_xm, g_ym, g_dm, calculateMaxSteps(g_dm));
    }
  } else if (!btn2 && button2WasPressed) {
    button2WasPressed = false;
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
  tft.setRotation(0); // Portrait orientation (170x320)

  // Turn on backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  Serial.printf("TFT dimensions after rotation: %d x %d\n", tft.width(), tft.height());
  Serial.printf("Expected dimensions: %d x %d\n", WIDTH, HEIGHT);

  // Initial render with current global parameters
  renderMandelbrot(g_xm, g_ym, g_dm, calculateMaxSteps(g_dm));

  Serial.println("Mandelbrot rendering complete!");
  Serial.println("Button controls:");
  Serial.println("  Left button: Open zoom menu, cycle through zoom options, right button executes");
  Serial.println("  Right button: Open direction menu, cycle through directions, left button executes");
}

void loop() {
  handleButtons();
  delay(10); // Small delay for button debouncing
}