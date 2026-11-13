/*
  Wind tunnel display: fan supply voltage -> test-section wind speed, airflow, fan RPM
  Neumorphic white/blue interface on the built-in LCD.

  Board:     ideaspark ESP32 (ESP-WROOM-32, CH340C) with built-in 1.9" ST7789 LCD (170x320)
  Arduino:   Tools > Board > esp32 > "ESP32 Dev Module"
  Libraries: "Adafruit ST7735 and ST7789 Library" + "Adafruit GFX Library" (+ "Adafruit BusIO")
             WiFi / WebServer / SPI come with the ESP32 board package

  Wiring:
    Fan supply +V (fan side of 5 A fuse) -> 47k -> node -> 10k -> GND
    node -> GPIO34 (D34), 0.1 uF from node to GND
    LED line constant +5 V -> ESP32 VIN   (unplug this wire while USB is plugged in)
    Fan supply -, LED line -, ESP32 GND all tied together (common ground)
*/

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Fonts/FreeSans9pt7b.h>       // Helvetica-style fonts that ship with Adafruit GFX
#include <Fonts/FreeSansBold18pt7b.h>

// =====================================================================
//  TEST MODE
//  true  = no sensor needed. Set a test voltage from:
//          - phone/laptop: join WiFi "FanDisplay-Test" (password 12345678),
//            open http://192.168.4.1 and drag the slider
//          - Serial Monitor (115200 baud): type a voltage, e.g. 7.5, press Enter
//  false = normal operation, reads the real fan supply on D34
// =====================================================================
#define TEST_MODE false

const char* TEST_SSID     = "FanDisplay-Test";
const char* TEST_PASSWORD = "12345678";  // at least 8 characters

// =====================================================================
//  Display pins (common ideaspark 1.9" mapping; if the screen stays blank,
//  compare with the demo sketch that came with your board)
// =====================================================================
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST   4
#define TFT_BL   32
const uint32_t SPI_HZ = 27000000;  // drop to 27000000 if you see glitches

// =====================================================================
//  Voltage sensing
// =====================================================================
const int   SENSE_PIN     = 34;                               // ADC1, input-only
const float DIVIDER_RATIO = (47000.0 + 10000.0) / 10000.0;    // 47k over 10k = 5.7
float       CAL_FACTOR    = 1.00;  // = multimeter volts / displayed volts
const int   SAMPLES       = 64;    // ADC readings averaged per update
const float EMA_ALPHA     = 0.2;   // extra smoothing (lower = smoother)

// The ESP32 ADC is unreliable below ~0.15 V at the pin (~0.85 V fan supply).
// Below ZERO_ON_V show 0; must rise above ZERO_OFF_V to show again.
const float ZERO_ON_V  = 0.85;
const float ZERO_OFF_V = 1.00;

// =====================================================================
//  Fan data (per fan) + (0 V, 0). Smooth monotone curve (PCHIP) between points.
// =====================================================================
const float TABLE_V[] = { 0.0,    3.0,    5.0,   10.0,   12.0 };  // volts
const float FAN_CFM[] = { 0.0,   46.0,   56.8,   85.0,   95.8 };  // CFM per fan
const float FAN_RPM[] = { 0.0, 1000.0, 1500.0, 2600.0, 2800.0 };  // RPM
const int   TABLE_N   = sizeof(TABLE_V) / sizeof(TABLE_V[0]);
const int   NUM_FANS  = 4;

// Tunnel: 240 x 240 mm fan section contracting to a 120 x 120 mm test section.
// Continuity: wind speed = total airflow / test-section area.
const float TEST_AREA_M2    = 0.120 * 0.120;
float       FLOW_EFFICIENCY = 1.00;  // = anemometer reading / displayed wind speed
const float CFM_TO_M3S      = 0.000471947;

// =====================================================================
//  Look: neumorphism, white + blue
// =====================================================================
struct Col { float r, g, b; };
// Background shape: circle, ring (circle outline) or rounded rectangle.
// (Defined up here because the Arduino IDE inserts function declarations near the top.)
enum ShapeType : uint8_t { CIRCLE, RING, RRECT };
struct Shape { ShapeType type; float a, b, c, d, e; };  // circle/ring: cx, cy, r, halfwidth | rrect: x0, y0, x1, y1, radius
const Col BG          = { 233, 237, 243 };   // soft white
const Col SHADE_DARK  = { 163, 177, 198 };   // shadow (bottom-right)
const Col SHADE_LIGHT = { 255, 255, 255 };   // highlight (top-left)
const Col HUB_FACE    = { 247, 249, 252 };
const Col GROOVE      = { 224, 229, 237 };
const Col BLUE        = {  37,  99, 235 };
const Col BLUE_LIGHT  = { 110, 175, 255 };
const Col CYAN        = {  45, 200, 240 };

#define rgb(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
const uint16_t COL_LABEL = rgb(132, 144, 165);
const uint16_t COL_UNIT  = rgb(118, 131, 155);
const uint16_t COL_VALUE = rgb( 29,  86, 225);
const uint16_t COL_WARN  = rgb(225,  65,  75);
const uint16_t COL_TEST  = rgb(235, 135,  45);

// Layout (landscape 320 x 170)
const int   FAN_CX = 64, FAN_CY = TEST_MODE ? 78 : 85;   // fan dial centre
const float DISC_R = 54;     // raised dial
const float WELL_R = 44;     // pressed-in well holding the fan
const float ARC_R  = 49;     // wind-speed arc (centre line) on the dial rim
const float ARC_HW = 2.0;    // arc half-width
const float HUB_R  = 10;     // hub
const float TIP_R  = 40;     // blade tips
const int   FAN_SZ = 84;     // fan canvas (covers r <= 42)
const int   FAN_N  = FAN_SZ * FAN_SZ;
const int   FAN_X0 = FAN_CX - FAN_SZ / 2, FAN_Y0 = FAN_CY - FAN_SZ / 2;

const float PANEL_X0 = 128, PANEL_Y0 = 9, PANEL_X1 = 311, PANEL_Y1 = 163, PANEL_RAD = 12;
const int   ROW_TOP[3] = { 9, 60, 111 };          // three rows inside the panel
const int   DIVIDER_Y[2] = { 60, 111 };          // engraved lines between rows
const int   LABEL_X = 142;
const int   STRIP_X = 140, STRIP_W = 152, STRIP_H = 31;   // value + unit area
const int   STRIP_BASE = 26;                     // text baseline inside the strip
const int   VALUE_RIGHT = 228;                   // values right-aligned here, units after
const char* ROW_LABEL[3] = { "WIND SPEED", "AIRFLOW", "FAN SPEED" };

const int INFO_X = 4, INFO_Y = 146, INFO_W = 120, INFO_H = 22;  // test-mode note

// Fan animation: on-screen spin speed is proportional to wind speed.
const int      BLADES      = 5;
const float    MAX_VIS_RPS = 1.2;   // icon revolutions/s at full wind (kept low so it doesn't strobe)
const float    SWEEP       = 0.9;   // blade curvature (radians from root to tip)
const uint32_t FRAME_MS    = 33;    // ~30 fps
const uint32_t SENSOR_MS   = TEST_MODE ? 50 : 200;

// =====================================================================
//  Globals
// =====================================================================
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
WebServer server(80);

GFXcanvas16* fanCanvas  = nullptr;
GFXcanvas16* valCanvas  = nullptr;
GFXcanvas16* infoCanvas = nullptr;
uint16_t* fanBg  = nullptr;   // background behind the fan (dial, well, hub)
uint16_t* infoBg = nullptr;
uint16_t* fanAng = nullptr;   // per-pixel angle (0..65535 = 0..2pi), precomputed
uint16_t* fanRad = nullptr;   // per-pixel radius * 64, precomputed
uint16_t  BG565;

float cfmSlope[TABLE_N], rpmSlope[TABLE_N];
float filteredVolts = 0, testVolts = 0;
bool  showingZero = true;
float lastWind = 0, lastFlow = 0, lastRpm = 0, windAtMax = 1;
float fanAngle = 0, visRps = 0;
int   shownArc = -1;
uint32_t lastFrameMs = 0, lastSensorMs = 0;
char  shownText[3][16] = { "", "", "" };
uint16_t shownColour[3] = { 0, 0, 0 };
char  shownInfo[24] = "";

// =====================================================================
//  Curve fit (monotone cubic / PCHIP, same as scipy PchipInterpolator)
// =====================================================================
static float sgn(float a) { return (a > 0) - (a < 0); }

static float pchipEndSlope(float h0, float h1, float d0, float d1) {
  float m = ((2 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
  if (sgn(m) != sgn(d0)) m = 0;
  else if (sgn(d0) != sgn(d1) && fabsf(m) > 3 * fabsf(d0)) m = 3 * d0;
  return m;
}

void pchipSlopes(const float* x, const float* y, int n, float* m) {  // 3 <= n <= 16
  float h[15], d[15];
  for (int k = 0; k < n - 1; k++) { h[k] = x[k + 1] - x[k]; d[k] = (y[k + 1] - y[k]) / h[k]; }
  for (int k = 1; k < n - 1; k++) {
    if (d[k - 1] * d[k] <= 0) { m[k] = 0; continue; }
    float w1 = 2 * h[k] + h[k - 1], w2 = h[k] + 2 * h[k - 1];
    m[k] = (w1 + w2) / (w1 / d[k - 1] + w2 / d[k]);
  }
  m[0]     = pchipEndSlope(h[0], h[1], d[0], d[1]);
  m[n - 1] = pchipEndSlope(h[n - 2], h[n - 3], d[n - 2], d[n - 3]);
}

float pchipEval(const float* x, const float* y, const float* m, int n, float v) {
  if (v <= x[0])     return y[0];
  if (v >= x[n - 1]) return y[n - 1];
  int i = 0;
  while (v > x[i + 1]) i++;
  float h = x[i + 1] - x[i], t = (v - x[i]) / h, t2 = t * t, t3 = t2 * t;
  return (2 * t3 - 3 * t2 + 1) * y[i] + (t3 - 2 * t2 + t) * h * m[i]
       + (-2 * t3 + 3 * t2) * y[i + 1] + (t3 - t2) * h * m[i + 1];
}

// Wind speed (m/s), airflow (m^3/min) and RPM for a supply voltage
void fanModel(float volts, float& wind, float& flowM3min, float& rpm) {
  float totalCfm = pchipEval(TABLE_V, FAN_CFM, cfmSlope, TABLE_N, volts) * NUM_FANS * FLOW_EFFICIENCY;
  float flowM3s  = totalCfm * CFM_TO_M3S;
  wind      = flowM3s / TEST_AREA_M2;
  flowM3min = flowM3s * 60.0;
  rpm       = pchipEval(TABLE_V, FAN_RPM, rpmSlope, TABLE_N, volts);
}

// =====================================================================
//  Sensor
// =====================================================================
float readSupplyVolts() {
  uint32_t sum_mV = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum_mV += analogReadMilliVolts(SENSE_PIN);  // factory-calibrated ADC reading
    delayMicroseconds(200);
  }
  return (sum_mV / (float)SAMPLES) / 1000.0 * DIVIDER_RATIO * CAL_FACTOR;
}

// =====================================================================
//  Colour helpers
// =====================================================================
static inline float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
static inline Col mix(const Col& a, const Col& b, float t) {
  return { a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t };
}
static const uint8_t BAYER[4][4] = { {0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5} };

// Float colour -> RGB565, with ordered dithering so soft shadows don't band
static uint16_t pack565(const Col& c, int x, int y) {
  float th = (BAYER[y & 3][x & 3] + 0.5f) / 16.0f;
  int r = (int)(c.r * 31.0f / 255.0f + th), g = (int)(c.g * 63.0f / 255.0f + th), b = (int)(c.b * 31.0f / 255.0f + th);
  r = r > 31 ? 31 : r; g = g > 63 ? 63 : g; b = b > 31 ? 31 : b;
  return (r << 11) | (g << 5) | b;
}
static uint16_t packFlat(const Col& c) {  // no dithering (flat areas)
  int r = (int)(c.r * 31.0f / 255.0f + 0.5f), g = (int)(c.g * 63.0f / 255.0f + 0.5f), b = (int)(c.b * 31.0f / 255.0f + 0.5f);
  return (r << 11) | (g << 5) | b;
}
static inline void unpack565(uint16_t c, float& r, float& g, float& b) {
  r = ((c >> 11) & 31) * (255.0f / 31); g = ((c >> 5) & 63) * (255.0f / 63); b = (c & 31) * (255.0f / 31);
}
static uint16_t blend565(uint16_t under, const Col& over, float a) {
  float r, g, b;
  unpack565(under, r, g, b);
  Col c = { r + (over.r - r) * a, g + (over.g - g) * a, b + (over.b - b) * a };
  return packFlat(c);
}

// =====================================================================
//  Neumorphic background: signed-distance shapes with soft shadows.
//  Rendered once at start-up (and for the backgrounds behind animated parts).
// =====================================================================
static float sdCircle(float x, float y, float cx, float cy, float r) {
  return sqrtf((x - cx) * (x - cx) + (y - cy) * (y - cy)) - r;
}
static float sdRRect(float x, float y, float x0, float y0, float x1, float y1, float rad) {
  float cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  float qx = fabsf(x - cx) - ((x1 - x0) / 2 - rad), qy = fabsf(y - cy) - ((y1 - y0) / 2 - rad);
  float ox = qx > 0 ? qx : 0, oy = qy > 0 ? qy : 0;
  float inside = qx > qy ? qx : qy;
  return sqrtf(ox * ox + oy * oy) + (inside < 0 ? inside : 0) - rad;
}
static inline float dropFall(float d, float blur)  { if (d <= 0) return 1; if (d >= blur) return 0; float t = 1 - d / blur; return t * t; }
static inline float innerFall(float d, float blur) { if (d >= 0) return 1; if (d <= -blur) return 0; float t = 1 + d / blur; return t * t; }

static float sdf(const Shape& s, float x, float y) {
  switch (s.type) {
    case CIRCLE: return sdCircle(x, y, s.a, s.b, s.c);
    case RING:   return fabsf(sdCircle(x, y, s.a, s.b, s.c)) - s.d;
    default:     return sdRRect(x, y, s.a, s.b, s.c, s.d, s.e);
  }
}

// Raised shape: highlight top-left, shadow bottom-right, flat face
static void raised(Col& c, const Shape& sh, float x, float y, float s, float blur, const Col& face, float k) {
  float d = sdf(sh, x, y), cov = clamp01(0.5f - d);
  if (cov < 1) {
    float dk = dropFall(sdf(sh, x - s, y - s), blur), lt = dropFall(sdf(sh, x + s, y + s), blur);
    if (dk > 0) c = mix(c, SHADE_DARK, dk * 0.62f * k);
    if (lt > 0) c = mix(c, SHADE_LIGHT, lt * 0.95f * k);
  }
  if (cov > 0) c = mix(c, face, cov);
}
// Pressed-in shape: inner shadow top-left, inner highlight bottom-right
static void inset(Col& c, const Shape& sh, float x, float y, float s, float blur, const Col& face, float k) {
  float d = sdf(sh, x, y), cov = clamp01(0.5f - d);
  if (cov <= 0) return;
  Col in = face;
  float dk = innerFall(sdf(sh, x - s, y - s), blur), lt = innerFall(sdf(sh, x + s, y + s), blur);
  if (dk > 0) in = mix(in, SHADE_DARK, dk * 0.55f * k);
  if (lt > 0) in = mix(in, SHADE_LIGHT, lt * 0.85f * k);
  c = mix(c, in, cov);
}

const Shape DIAL   = { CIRCLE, (float)FAN_CX, (float)FAN_CY, DISC_R, 0, 0 };
const Shape GROOVE_RING = { RING, (float)FAN_CX, (float)FAN_CY, ARC_R, ARC_HW, 0 };
const Shape WELL   = { CIRCLE, (float)FAN_CX, (float)FAN_CY, WELL_R, 0, 0 };
const Shape HUB    = { CIRCLE, (float)FAN_CX, (float)FAN_CY, HUB_R, 0, 0 };
const Shape PANEL  = { RRECT, PANEL_X0, PANEL_Y0, PANEL_X1, PANEL_Y1, PANEL_RAD };

uint16_t scenePixel(int px, int py) {
  float x = px + 0.5f, y = py + 0.5f;
  Col c = BG;
  raised(c, DIAL, x, y, 5, 11, BG, 1);
  inset (c, GROOVE_RING, x, y, 1, 2.5f, GROOVE, 0.8f);
  inset (c, WELL, x, y, 4, 9, BG, 1);
  raised(c, HUB, x, y, 2, 4.5f, HUB_FACE, 0.8f);
  float dDot = sdCircle(x, y, FAN_CX, FAN_CY, 3.2f);
  if (dDot < 0.5f) c = mix(c, BLUE, clamp01(0.5f - dDot));   // blue centre cap
  raised(c, PANEL, x, y, 5, 11, BG, 1);
  for (int i = 0; i < 2; i++) {   // engraved dividers between the rows
    if (px >= LABEL_X && px <= PANEL_X1 - 14) {
      if (py == DIVIDER_Y[i])     c = mix(c, SHADE_DARK, 0.35f);
      if (py == DIVIDER_Y[i] + 1) c = mix(c, SHADE_LIGHT, 0.9f);
    }
  }
  bool flat = fabsf(c.r - BG.r) < 0.4f && fabsf(c.g - BG.g) < 0.4f && fabsf(c.b - BG.b) < 0.4f;
  return flat ? BG565 : pack565(c, px, py);
}

void renderSceneInto(uint16_t* buf, int x0, int y0, int w, int h) {
  for (int j = 0; j < h; j++)
    for (int i = 0; i < w; i++) buf[j * w + i] = scenePixel(x0 + i, y0 + j);
}

// =====================================================================
//  Drawing
// =====================================================================
void drawStatic() {
  uint16_t line[320];
  for (int y = 0; y < 170; y++) {
    for (int x = 0; x < 320; x++) line[x] = scenePixel(x, y);
    tft.drawRGBBitmap(0, y, line, 320, 1);
  }
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(COL_LABEL);
  for (int r = 0; r < 3; r++) {
    tft.setCursor(LABEL_X, ROW_TOP[r] + 16);
    tft.print(ROW_LABEL[r]);
  }
}

// Tiny superscript "3" (5 x 7 pixels)
static const uint8_t SUP3[7] = { 0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E };
void drawSup3(GFXcanvas16* c, int x, int top, uint16_t col) {
  for (int r = 0; r < 7; r++)
    for (int b = 0; b < 5; b++)
      if (SUP3[r] & (0x10 >> b)) c->drawPixel(x + b, top + r, col);
}

void drawUnit(GFXcanvas16* c, int x, int base, int unit) {
  c->setFont(&FreeSans9pt7b);
  c->setTextColor(COL_UNIT);
  c->setCursor(x, base);
  if (unit == 0) { c->print("m/s"); return; }
  if (unit == 2) { c->print("rpm"); return; }
  c->print("m");
  int sx = c->getCursorX() + 1;
  drawSup3(c, sx, base - 14, COL_UNIT);   // m^3
  c->setCursor(sx + 7, base);
  c->print("/min");
}

// Redraws one reading (value + unit) only when it has changed
void drawReading(int row, const char* text, uint16_t colour) {
  if (strcmp(text, shownText[row]) == 0 && colour == shownColour[row]) return;
  strncpy(shownText[row], text, sizeof(shownText[row]) - 1);
  shownColour[row] = colour;

  int base = ROW_TOP[row] + 46;
  valCanvas->fillScreen(BG565);   // the panel face is flat background colour
  valCanvas->setFont(&FreeSansBold18pt7b);
  int16_t bx, by; uint16_t bw, bh;
  valCanvas->getTextBounds(text, 0, STRIP_BASE, &bx, &by, &bw, &bh);
  valCanvas->setCursor(VALUE_RIGHT - STRIP_X - (bx + bw), STRIP_BASE);  // right-align
  valCanvas->setTextColor(colour);
  valCanvas->print(text);
  drawUnit(valCanvas, VALUE_RIGHT - STRIP_X + 5, STRIP_BASE, row);
  tft.drawRGBBitmap(STRIP_X, base - STRIP_BASE, valCanvas->getBuffer(), STRIP_W, STRIP_H);
}

// Blue arc on the dial rim showing wind speed as a fraction of full speed
void drawArc(float frac) {
  int q = (int)(clamp01(frac) * 400 + 0.5f);
  if (q == shownArc) return;
  shownArc = q;
  float sweep = q / 400.0f * TWO_PI;
  int r0 = (int)(ARC_R - ARC_HW - 2), r1 = (int)(ARC_R + ARC_HW + 2);
  tft.startWrite();
  for (int y = FAN_CY - r1; y <= FAN_CY + r1; y++) {
    for (int x = FAN_CX - r1; x <= FAN_CX + r1; x++) {
      float dx = x + 0.5f - FAN_CX, dy = y + 0.5f - FAN_CY, r = sqrtf(dx * dx + dy * dy);
      if (r < r0 || r > r1) continue;
      float a = atan2f(dy, dx) + HALF_PI;               // 0 at 12 o'clock, clockwise
      if (a < 0) a += TWO_PI;
      float radCov = clamp01(ARC_HW + 0.5f - fabsf(r - ARC_R));
      float angCov = (sweep <= 0) ? 0 : clamp01(fminf(a * r, (sweep - a) * r) + 0.5f);
      uint16_t bg = scenePixel(x, y);
      float cov = radCov * angCov;
      Col blue = mix(BLUE, CYAN, a / TWO_PI);
      uint16_t out = cov > 0.004f ? blend565(bg, blue, cov) : bg;
      tft.writePixel(x, y, out);
      int fx = x - FAN_X0, fy = y - FAN_Y0;   // the fan canvas corners overlap the arc:
      if (fx >= 0 && fy >= 0 && fx < FAN_SZ && fy < FAN_SZ) fanBg[fy * FAN_SZ + fx] = out;  // keep it in the fan's background
    }
  }
  tft.endWrite();
}

// Blade coverage (0..1) at fan-canvas pixel k; u = position across the blade (0..1)
static float bladeCov(int k, uint16_t rot16, float& u) {
  float r = fanRad[k] * (1.0f / 64);
  if (r < HUB_R - 0.5f || r > TIP_R + 0.5f) return 0;
  float rn = (r - HUB_R) / (TIP_R - HUB_R);
  uint16_t sweep16 = (uint16_t)(rn * SWEEP * (65536.0f / TWO_PI));
  uint16_t t16 = (uint16_t)((uint16_t)(fanAng[k] - rot16 - sweep16) * BLADES);  // position within one blade period
  float w = (0.30f + 0.20f * rn) * 65536.0f;                                     // blade width
  float pxPer = (TWO_PI / BLADES) / 65536.0f * r;
  float cov = (t16 < w) ? clamp01(fminf(t16, w - t16) * pxPer + 0.5f)
                        : clamp01(0.5f - fminf(t16 - w, 65536.0f - t16) * pxPer);
  cov *= fminf(clamp01(r - HUB_R + 0.5f), clamp01(TIP_R - r + 0.5f));
  u = clamp01(t16 / w);
  return cov;
}

// Smooth, anti-aliased fan with soft blade shadows, rotated by fanAngle
void renderFan() {
  uint16_t* dst = fanCanvas->getBuffer();
  memcpy(dst, fanBg, FAN_N * sizeof(uint16_t));
  uint16_t rot16 = (uint16_t)(fanAngle * (65536.0f / TWO_PI));
  const int SH = 2;                        // shadow offset (down-right)
  for (int y = 0; y < FAN_SZ; y++) {
    for (int x = 0; x < FAN_SZ; x++) {
      int k = y * FAN_SZ + x;
      float u, su;
      float cov = bladeCov(k, rot16, u);
      float sh = (x >= SH && y >= SH) ? bladeCov(k - SH * FAN_SZ - SH, rot16, su) : 0;
      if (sh > 0 && cov < 1) dst[k] = blend565(dst[k], SHADE_DARK, sh * 0.45f * (1 - cov));
      if (cov > 0.003f) {
        float rn = clamp01((fanRad[k] * (1.0f / 64) - HUB_R) / (TIP_R - HUB_R));
        Col c = mix(BLUE_LIGHT, BLUE, u);           // light leading edge -> deep blue
        c = mix(c, BLUE_LIGHT, 0.25f * rn);         // a touch lighter towards the tips
        dst[k] = blend565(dst[k], c, cov);
      }
    }
  }
  tft.drawRGBBitmap(FAN_X0, FAN_Y0, dst, FAN_SZ, FAN_SZ);
}

// Test-mode note under the fan
void drawInfo(const char* text) {
  if (strcmp(text, shownInfo) == 0) return;
  strncpy(shownInfo, text, sizeof(shownInfo) - 1);
  memcpy(infoCanvas->getBuffer(), infoBg, INFO_W * INFO_H * sizeof(uint16_t));
  infoCanvas->setFont(&FreeSans9pt7b);
  infoCanvas->setTextColor(COL_TEST);
  int16_t bx, by; uint16_t bw, bh;
  infoCanvas->getTextBounds(text, 0, 16, &bx, &by, &bw, &bh);
  infoCanvas->setCursor((INFO_W - bw) / 2 - bx, 16);   // centred under the dial
  infoCanvas->print(text);
  tft.drawRGBBitmap(INFO_X, INFO_Y, infoCanvas->getBuffer(), INFO_W, INFO_H);
}

// =====================================================================
//  Readings
// =====================================================================
void updateReadings() {
  if (showingZero && filteredVolts > ZERO_OFF_V)      showingZero = false;
  else if (!showingZero && filteredVolts < ZERO_ON_V) showingZero = true;
  float volts = showingZero ? 0.0 : filteredVolts;
  bool  aboveRange = volts > TABLE_V[TABLE_N - 1] + 0.2;  // values held at the 12 V figures

  float wind, flow, rpm;
  fanModel(volts, wind, flow, rpm);
  lastWind = wind; lastFlow = flow; lastRpm = rpm;

  char s[24];
  uint16_t col = aboveRange ? COL_WARN : COL_VALUE;
  snprintf(s, sizeof(s), "%.2f", wind); drawReading(0, s, col);
  snprintf(s, sizeof(s), "%.2f", flow); drawReading(1, s, col);
  snprintf(s, sizeof(s), "%.0f", rpm);  drawReading(2, s, col);
  drawArc(windAtMax > 0 ? wind / windAtMax : 0);

  if (TEST_MODE) {
    snprintf(s, sizeof(s), "TEST  %.2f V", testVolts);
    drawInfo(s);
  } else {
    // raw = measured voltage before the zero cut-off (use for CAL_FACTOR)
    Serial.printf("raw=%.2f V  used=%.2f V  wind=%.2f m/s  flow=%.2f m3/min  rpm=%.0f\n",
                  filteredVolts, volts, wind, flow, rpm);
  }
}

void animateFan(float dt) {
  float target = (windAtMax > 0) ? MAX_VIS_RPS * lastWind / windAtMax : 0;
  visRps += (target - visRps) * fminf(1.0f, dt * 3.0f);  // ease spin up / down
  if (target == 0 && visRps < 0.01f) visRps = 0;
  fanAngle += visRps * TWO_PI * dt;
  if (fanAngle > TWO_PI) fanAngle -= TWO_PI;
  renderFan();
}

// =====================================================================
//  Test mode: web slider + Serial input
// =====================================================================
const char TEST_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Fan display test</title>
<style>
body{font-family:Helvetica,Arial,sans-serif;background:#e9edf3;color:#5b6b85;max-width:440px;margin:24px auto;padding:0 16px}
.card{background:#e9edf3;border-radius:22px;padding:22px;box-shadow:8px 8px 18px #a3b1c6,-8px -8px 18px #fff}
h2{font-weight:500;margin:0 0 6px}
.v{font-size:40px;color:#1d56e1;margin:6px 0 14px}
input[type=range]{width:100%;accent-color:#2563eb;height:36px}
table{width:100%;margin-top:14px}td{padding:6px 0;font-size:19px}td.n{color:#1d56e1;text-align:right;padding-right:8px}
</style></head><body><div class="card">
<h2>Fan display test</h2>
<div class="v"><span id="v">0.00</span> V</div>
<input type="range" id="s" min="0" max="13" step="0.05" value="0">
<table>
<tr><td>Wind speed</td><td class="n" id="w">0.00</td><td>m/s</td></tr>
<tr><td>Airflow</td><td class="n" id="f">0.00</td><td>m&sup3;/min</td></tr>
<tr><td>Fan speed</td><td class="n" id="r">0</td><td>rpm</td></tr>
</table></div>
<script>
const s=document.getElementById('s');let busy=false,pending=null;
function send(v){
  if(busy){pending=v;return;}busy=true;
  fetch('/set?v='+v).then(r=>r.json()).then(d=>{
    document.getElementById('w').textContent=d.wind.toFixed(2);
    document.getElementById('f').textContent=d.flow.toFixed(2);
    document.getElementById('r').textContent=d.rpm.toFixed(0);
  }).catch(()=>{}).finally(()=>{busy=false;if(pending!==null){const p=pending;pending=null;send(p);}});
}
s.oninput=()=>{document.getElementById('v').textContent=(+s.value).toFixed(2);send(s.value);};
</script></body></html>
)HTML";

void handleRoot() { server.send_P(200, "text/html", TEST_PAGE); }

void handleSet() {
  if (server.hasArg("v")) testVolts = constrain(server.arg("v").toFloat(), 0.0f, 15.0f);
  filteredVolts = testVolts;
  updateReadings();  // update the LCD first so the page gets the same numbers
  char json[96];
  snprintf(json, sizeof(json), "{\"v\":%.2f,\"wind\":%.2f,\"flow\":%.2f,\"rpm\":%.0f}",
           testVolts, lastWind, lastFlow, lastRpm);
  server.send(200, "application/json", json);
}

void checkSerialInput() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() > 0 && (isdigit(line[0]) || line[0] == '.')) {  // ignore non-numbers
    testVolts = constrain(line.toFloat(), 0.0f, 15.0f);
    Serial.printf("Test voltage set to %.2f V\n", testVolts);
  }
}

// =====================================================================
//  Setup / loop
// =====================================================================
void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetPinAttenuation(SENSE_PIN, ADC_11db);  // ~0-3.1 V input range

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft.init(170, 320);
  tft.setSPISpeed(SPI_HZ);
  tft.setRotation(1);  // landscape; use 3 if it's upside down

  BG565 = packFlat(BG);
  fanCanvas  = new GFXcanvas16(FAN_SZ, FAN_SZ);
  valCanvas  = new GFXcanvas16(STRIP_W, STRIP_H);
  infoCanvas = new GFXcanvas16(INFO_W, INFO_H);
  fanBg  = (uint16_t*)malloc(FAN_N * sizeof(uint16_t));
  fanAng = (uint16_t*)malloc(FAN_N * sizeof(uint16_t));
  fanRad = (uint16_t*)malloc(FAN_N * sizeof(uint16_t));
  infoBg = (uint16_t*)malloc(INFO_W * INFO_H * sizeof(uint16_t));
  if (!fanCanvas->getBuffer() || !valCanvas->getBuffer() || !infoCanvas->getBuffer() ||
      !fanBg || !fanAng || !fanRad || !infoBg) {
    Serial.println("Not enough memory for display buffers");
    while (true) delay(1000);
  }

  // Precompute polar coordinates of every fan pixel (makes each frame fast)
  for (int y = 0; y < FAN_SZ; y++) {
    for (int x = 0; x < FAN_SZ; x++) {
      float dx = FAN_X0 + x + 0.5f - FAN_CX, dy = FAN_Y0 + y + 0.5f - FAN_CY;
      float a = atan2f(dy, dx);
      if (a < 0) a += TWO_PI;
      fanAng[y * FAN_SZ + x] = (uint16_t)(a * (65536.0f / TWO_PI));
      fanRad[y * FAN_SZ + x] = (uint16_t)(sqrtf(dx * dx + dy * dy) * 64);
    }
  }

  pchipSlopes(TABLE_V, FAN_CFM, TABLE_N, cfmSlope);
  pchipSlopes(TABLE_V, FAN_RPM, TABLE_N, rpmSlope);
  float f, r;
  fanModel(TABLE_V[TABLE_N - 1], windAtMax, f, r);  // wind speed at 12 V = full spin, full arc

  drawStatic();
  renderSceneInto(fanBg, FAN_X0, FAN_Y0, FAN_SZ, FAN_SZ);
  renderSceneInto(infoBg, INFO_X, INFO_Y, INFO_W, INFO_H);

  if (TEST_MODE) {
    WiFi.softAP(TEST_SSID, TEST_PASSWORD);
    server.on("/", handleRoot);
    server.on("/set", handleSet);
    server.begin();
    Serial.setTimeout(50);
    Serial.printf("\nTEST MODE: join WiFi \"%s\" (password %s), open http://%s\n",
                  TEST_SSID, TEST_PASSWORD, WiFi.softAPIP().toString().c_str());
    Serial.println("Or type a voltage here (e.g. 7.5) and press Enter.");
    filteredVolts = testVolts;
  } else {
    filteredVolts = readSupplyVolts();
  }
  updateReadings();
  renderFan();
  lastFrameMs = lastSensorMs = millis();
}

void loop() {
  if (TEST_MODE) {
    server.handleClient();
    checkSerialInput();
  }

  uint32_t now = millis();

  if (now - lastSensorMs >= SENSOR_MS) {
    lastSensorMs = now;
    if (TEST_MODE) {
      filteredVolts = testVolts;  // no sensor, no smoothing
    } else {
      float v = readSupplyVolts();
      filteredVolts = EMA_ALPHA * v + (1.0 - EMA_ALPHA) * filteredVolts;
    }
    updateReadings();
  }

  if (now - lastFrameMs >= FRAME_MS) {
    animateFan((now - lastFrameMs) / 1000.0f);
    lastFrameMs = now;
  }
}

