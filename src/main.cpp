#include <Arduino.h>
#include <TFT_eSPI.h>

// Rendering mode: set to 1 to render while calculating, 0 to calculate then render
#define RENDER_WHILE_CALCULATING 1

// Algorithm mode: set to 1 for optimized recursive algorithm, 0 for naive nested loop
#define USE_OPTIMIZED_ALGORITHM 1

TFT_eSPI tft = TFT_eSPI();

// Display dimensions in landscape mode
#define WIDTH 320
#define HEIGHT 170

// Grid to store Mandelbrot iteration counts
uint16_t grid[WIDTH][HEIGHT];

// Max steps for color mapping (set during renderMandelbrot)
int g_maxSteps = 100;

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
  uint16_t color = stepsToColor(value, g_maxSteps);
  tft.drawPixel(xi, yi, color);
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
      if (n1 == n2 && n1 < maxSteps) {
        // 2 less than ny, because we already set first and last
        // Only fill if not in the set (n1 < maxSteps) to avoid artifacts
        for (int ni = 2; ni < ny; ni++) {
          yi++;
          setGridPixel(xi, yi, n1);
        }
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
      if (n1 == n2 && n1 < maxSteps) {
        // 2 less than nx, because we already set first and last
        // Only fill if not in the set (n1 < maxSteps) to avoid artifacts
        for (int ni = 2; ni < nx; ni++) {
          xi++;
          setGridPixel(xi, yi, n1);
        }
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

    if (n1 == n2 && n1 == n3 && n1 == n4 && n1 < maxSteps) {
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

    fillGrid(xi + 1, x + d, nx-2, yi + 1, y + d, ny-2, d, maxSteps);
	}
  while (false);
}

// Calculate and render Mandelbrot set
// xm, ym = top-left corner of window
// dm = width of window (height = dm * 17/32)
void renderMandelbrot(float xm, float ym, float dm, int maxSteps) {
  unsigned long startTime = millis();
  g_maxSteps = maxSteps;

  float window_height = dm * 17.0 / 32.0;
  Serial.printf("Calculating Mandelbrot: top-left=(%.4f, %.4f), width=%.4f, height=%.4f\n", xm, ym, dm, window_height);
  Serial.printf("Render mode: %s\n", RENDER_WHILE_CALCULATING ? "while-calculating" : "post-render");
  Serial.printf("Algorithm: %s\n", USE_OPTIMIZED_ALGORITHM ? "optimized-recursive" : "naive-loop");

  // Calculate the grid
  float scale = dm/320.0;

#if USE_OPTIMIZED_ALGORITHM
  // Optimized recursive algorithm
  fillGrid(0, xm, WIDTH, 0, ym, HEIGHT, scale, maxSteps);
#else
  // Naive nested loop algorithm
  for (int xi = 0; xi < WIDTH; xi++) {
    for (int yi = 0; yi < HEIGHT; yi++) {
      float x = xm + xi * scale;
      float y = ym + yi * scale;
      int n = nSteps(x, y, maxSteps);
      setGridPixel(xi, yi, n);
    }

    // Progress indicator every 32 columns
    if (xi % 32 == 0) {
      Serial.printf("Progress: %d%%\n", (xi * 100) / WIDTH);
    }
  }
#endif

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
}

void setup() {
  Serial.begin(115200);
  delay(1000);

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

  // Zoom into a detailed spiral region (seahorse valley)
  // Top-left at (-0.755, 0.095), width of 0.02
  // Intricate spirals with lots of color detail
  renderMandelbrot(-0.770, 0.095, 0.01, 1000);

  Serial.println("Mandelbrot rendering complete!");
}

void loop() {
  // Static display, nothing to do
}