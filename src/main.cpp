#include <Arduino.h>
#include <TFT_eSPI.h>

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

// Grid to store Mandelbrot iteration counts
uint16_t grid[WIDTH][HEIGHT];

// Max steps for color mapping (set during renderMandelbrot)
int g_curMaxSteps = 100;

// Current view parameters
float g_xm = -0.770;
float g_ym = 0.095;
float g_dm = 0.01;
bool g_calculating = false;
int g_maxSteps = 1000;

// Multi-core variables
SemaphoreHandle_t g_renderMutex = NULL;
volatile bool g_core1Done = false;

// Task parameters structure
struct RenderTaskParams {
  int startX;
  int endX;
  float xm;
  float ym;
  float dm;
  int maxSteps;
};

// Calculate number of steps for Mandelbrot iteration
int nSteps(float x0, float y0, int maxSteps) {
  float x = 0.0;
  float y = 0.0;
  int steps = 0;

  while (steps < maxSteps) {
    float x2 = x * x;
    float y2 = y * y;

    // Check if we've escaped (distance squared > 4)
    if (x2 + y2 > 4.0) {
      break;
    }

    // Mandelbrot iteration: z = z^2 + c
    float xtemp = x2 - y2 + x0;
    y = 2.0 * x * y + y0;
    x = xtemp;

    steps++;
  }

  return steps;
}

// Forward declaration
uint16_t stepsToColor(int steps, int maxSteps);

// Set grid value and optionally render immediately
void setGridPixel(int xi, int yi, uint16_t value) {
  grid[xi][yi] = value;
#if RENDER_WHILE_CALCULATING
  uint16_t color = stepsToColor(value, g_curMaxSteps);
  if (g_renderMutex != NULL) {
    xSemaphoreTake(g_renderMutex, portMAX_DELAY);
    tft.drawPixel(xi, yi, color);
    xSemaphoreGive(g_renderMutex);
  } else {
    tft.drawPixel(xi, yi, color);
  }
#endif
}

// Map iteration count to color
uint16_t stepsToColor(int steps, int maxSteps) {
  if (steps >= maxSteps) {
    return TFT_BLACK; // Inside the set
  }

  // Create a color gradient
  uint8_t hue = (steps * 255) / maxSteps;

  // Simple RGB mapping
  if (hue < 85) {
    uint8_t r = hue * 3;
    uint8_t g = 255 - hue * 3;
    uint8_t b = 0;
    return tft.color565(r, g, b);
  } else if (hue < 170) {
    hue -= 85;
    uint8_t r = 255 - hue * 3;
    uint8_t g = 0;
    uint8_t b = hue * 3;
    return tft.color565(r, g, b);
  } else {
    hue -= 170;
    uint8_t r = 0;
    uint8_t g = hue * 3;
    uint8_t b = 255 - hue * 3;
    return tft.color565(r, g, b);
  }
}

void fillGrid(int xi, float x, int nx, int yi, float y, int ny, float d, int maxSteps) {

  do {

    if (nx < 1) {
      break;
    }

    if (ny < 1) {
      break;
    }

    if (nx == 1 && ny == 1) {
      int n = nSteps(x, y, maxSteps);
      setGridPixel(xi, yi, n);
      break;
    }

	  if (nx == 1) {

  		int n1 = nSteps(x, y, maxSteps);
		  int n2 = nSteps(x, y+d*(ny-1), maxSteps);

		  setGridPixel(xi, yi, n1);
		  setGridPixel(xi, yi + ny - 1, n2);

      if (ny == 2) {
        break;
      }

		  int ny1 = (ny-2)/2;
		  int ny2 = ny-2-ny1;

      fillGrid(xi, x, 1, yi + 1, y + d, ny1, d, maxSteps);
      fillGrid(xi, x, 1, yi + 1 + ny1, y + d*(1 + ny1), ny2, d, maxSteps);
  		break;
	  }

    if (ny == 1) {

  		int n1 = nSteps(x, y, maxSteps);
		  int n2 = nSteps(x + d*(nx-1), y, maxSteps);

		  setGridPixel(xi, yi, n1);
		  setGridPixel(xi + nx - 1, yi, n2);

      if (nx == 2) {
        break;
      }

      int nx1 = (nx-2)/2;
      int nx2 = nx-2-nx1;

      fillGrid(xi + 1, x + d, nx1, yi, y, 1, d, maxSteps);
      fillGrid(xi + 1 + nx1, x + d*(1 + nx1), nx2, yi, y, 1, d, maxSteps);
  		break;
	  }

    int n1 = nSteps(x, y, maxSteps);
    int n2 = nSteps(x + d*(nx-1), y, maxSteps);
    int n3 = nSteps(x, y + d*(ny-1), maxSteps);
    int n4 = nSteps(x + d*(nx-1), y + d*(ny-1), maxSteps);

    if (n1 == n2 && n1 == n3 && n1 == n4 && (n1 < maxSteps)) {
      // Fill entire block
      // Only fill if not in the set (n1 < maxSteps) to avoid artifacts
      for (int xi2 = 0; xi2 < nx; xi2++) {
        for (int yi2 = 0; yi2 < ny; yi2++) {
          setGridPixel(xi + xi2, yi + yi2, n1);
        }
      }
      break;
    }

    setGridPixel(xi, yi, n1);
    setGridPixel(xi + nx - 1, yi, n2);
    setGridPixel(xi, yi + ny - 1, n3);
    setGridPixel(xi + nx - 1, yi + ny - 1, n4);

    fillGrid(xi + 1, x + d, nx-2, yi, y, 1, d, maxSteps);
    fillGrid(xi + 1, x + d, nx-2, yi + ny - 1, y + d*(ny-1), 1, d, maxSteps);

    fillGrid(xi, x, 1, yi + 1, y + d, ny-2, d, maxSteps);
    fillGrid(xi + nx - 1, x + d*(nx-1), 1, yi + 1, y + d, ny-2, d, maxSteps);

    int nx1 = (nx-2)/2;
    int nx2 = nx-2-nx1;

    int ny1 = (ny-2)/2;
    int ny2 = ny-2-ny1;

    fillGrid(xi + 1,       x + d,           nx1, yi + 1,       y + d,         ny1, d, maxSteps);
    fillGrid(xi + 1,       x + d,           nx1, yi + 1 + ny1, y + d + ny1*d, ny2, d, maxSteps);
    fillGrid(xi + 1 + nx1, x + d + nx1*d,   nx2, yi + 1,       y + d,         ny1, d, maxSteps);
    fillGrid(xi + 1 + nx1, x + d + nx1*d,   nx2, yi + 1 + ny1, y + d + ny1*d, ny2, d, maxSteps);
	}
  while (false);
}

// Partial fill for multi-core rendering
void fillGridPartial(int startX, int endX, float xm, float ym, float dm, int maxSteps) {
  float scale = dm / 320.0;
  int width = endX - startX;
  
#if USE_OPTIMIZED_ALGORITHM
  // Optimized recursive algorithm for this range
  fillGrid(startX, xm + startX * scale, width, 0, ym, HEIGHT, scale, maxSteps);
#else
  // Naive nested loop algorithm for this range
  for (int xi = startX; xi < endX; xi++) {
    for (int yi = 0; yi < HEIGHT; yi++) {
      float x = xm + xi * scale;
      float y = ym + yi * scale;
      int n = nSteps(x, y, maxSteps);
      setGridPixel(xi, yi, n);
    }
  }
#endif
}

// Task to run on core 1
void core1Task(void* params) {
  RenderTaskParams* p = (RenderTaskParams*)params;
  fillGridPartial(p->startX, p->endX, p->xm, p->ym, p->dm, p->maxSteps);
  g_core1Done = true;
  vTaskDelete(NULL);
}

// Calculate and render Mandelbrot set
// xm, ym = top-left corner of window
// dm = width of window (height = dm * 17/32)
void renderMandelbrot(float xm, float ym, float dm, int maxSteps) {
  g_calculating = true;
  unsigned long startTime = millis();
  g_curMaxSteps = maxSteps;

  float window_height = dm * 17.0 / 32.0;
  Serial.printf("Calculating Mandelbrot: top-left=(%.4f, %.4f), width=%.4f, height=%.4f\n", xm, ym, dm, window_height);
  Serial.printf("Render mode: %s\n", RENDER_WHILE_CALCULATING ? "while-calculating" : "post-render");
  Serial.printf("Algorithm: %s\n", USE_OPTIMIZED_ALGORITHM ? "optimized-recursive" : "naive-loop");
  Serial.println("Using dual-core rendering");

  // Split work between cores
  int midPoint = WIDTH / 2;
  
  // Setup task parameters for core 1
  RenderTaskParams taskParams;
  taskParams.startX = midPoint;
  taskParams.endX = WIDTH;
  taskParams.xm = xm;
  taskParams.ym = ym;
  taskParams.dm = dm;
  taskParams.maxSteps = maxSteps;
  
  g_core1Done = false;
  
  // Create task on core 1 (core 0 is typically used for WiFi/system tasks)
  xTaskCreatePinnedToCore(
    core1Task,           // Task function
    "Core1Render",       // Task name
    8192,                // Stack size
    &taskParams,         // Parameters
    1,                   // Priority
    NULL,                // Task handle
    1                    // Core ID
  );
  
  // Calculate left half on core 0
  fillGridPartial(0, midPoint, xm, ym, dm, maxSteps);
  
  // Wait for core 1 to finish
  while (!g_core1Done) {
    delay(1);
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
    renderMandelbrot(g_xm, g_ym, g_dm, g_maxSteps);
    button1PendingSingle = false;
    button1ReleaseTime = 0;
  }

  if (button2PendingSingle && (millis() - button2ReleaseTime >= DOUBLE_PRESS_TIME)) {
    Serial.println("Button 2 single press: Shift right");
    g_xm += g_dm / 2.0;
    renderMandelbrot(g_xm, g_ym, g_dm, g_maxSteps);
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
      renderMandelbrot(g_xm, g_ym, g_dm, g_maxSteps);
    } else if (button1DoubleInProgress) {
      // This was a double press
      Serial.println("Button 1 double press: Shift up");
      button1DoubleInProgress = false;
      button1PendingSingle = false;
      button1ReleaseTime = 0;
      g_ym -= g_dm / 2.0 * 17.0 / 32.0;
      renderMandelbrot(g_xm, g_ym, g_dm, g_maxSteps);
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

        renderMandelbrot(g_xm, g_ym, g_dm, g_maxSteps);
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
      renderMandelbrot(g_xm, g_ym, g_dm, g_maxSteps);
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

  // Initialize buttons
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);

  // Initialize the display
  tft.init();
  tft.setRotation(1); // Landscape orientation (320x170)

  // Turn on backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // Clear screen
  tft.fillScreen(TFT_BLACK);

  Serial.println("Display initialized!");
  Serial.printf("TFT dimensions after rotation: %d x %d\n", tft.width(), tft.height());
  Serial.printf("Expected dimensions: %d x %d\n", WIDTH, HEIGHT);

  // Initial render with current global parameters
  renderMandelbrot(g_xm, g_ym, g_dm, g_maxSteps);

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