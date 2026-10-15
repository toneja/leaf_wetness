/*
  Leaf wetness logger, v6
  -----------------------
  Hardware : RAK4631 core, PHYTOS31 leaf wetness sensor read through an
             Adafruit ADS1115 16-bit ADC, RAK1906 (BME680), RAK1921 OLED,
             RAK15002 microSD, u-blox GNSS module (I2C)
  Libraries: Adafruit ADS1X15, U8g2, Adafruit BME680 (+ Adafruit Unified Sensor),
             SparkFun u-blox GNSS; SD.h and bluefruit.h are built in

  Wiring:
    PHYTOS31  supply -> 3V3_S,  ground -> GND,  signal -> ADS1115 A0
    ADS1115   VDD -> 3V3_S,  GND -> GND,  SDA/SCL -> the WisBlock I2C bus (address 0x48)

  Wetness algorithm: modelled on Binks et al. (2021), "Measuring the vertical
  profile of leaf wetness in a forest canopy", MethodsX 8:101332. There are no
  fixed wet/dry thresholds. Instead the sensor is averaged into 15-minute steps
  and wetting is recognised by how fast the signal rises:

    RAIN  a rise of RAIN_RISE_MV or more in one step
    DEW   a slow rise (DEW_MIN_RISE_MV..DEW_MAX_RISE_MV) for DEW_MIN_STEPS steps in a row
    WET   a rise between the two (light rain, drizzle, fog: not in the paper)

  The reading just before the wetting began becomes that period's baseline,
  which makes the method immune to sensor drift and dirt. The leaf counts as dry
  again when the signal falls back to within 10 % of the baseline.

  Every SAMPLE_INTERVAL_MS the sketch reads the sensor and the air sensor, and
  sends one CSV row to the SD card and Bluetooth (and Serial when DEBUG is 1),
  and updates the OLED. The wet/dry state itself only changes every 15 minutes.

  GPS time only timestamps rows. Durations use the internal clock, so logging
  works (with blank date/time) before the GPS has a fix.
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <bluefruit.h>
#include <Adafruit_ADS1X15.h>
#include <U8g2lib.h>
#include <Adafruit_BME680.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>


// =============================================================================
//  Debug switch
// =============================================================================

// 1: use Serial (status messages and every CSV row, 115200 baud).
// 0: Serial is never started or used at all.
#define DEBUG 1

#if DEBUG
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)    ((void)0)
  #define DEBUG_PRINTLN(x)  ((void)0)
#endif


// =============================================================================
//  Configuration
// =============================================================================

// ---- Pins, addresses and names ----
const int     SENSOR_POWER_PIN     = WB_IO2;   // enables the 3V3_S power rail
const uint8_t BME680_I2C_ADDRESS   = 0x76;     // RAK1906
// Each boot starts a new log file: WET0001.CSV, WET0002.CSV, ... (8.3 names work on any SD library).
const char*   LOG_FILE_PATTERN     = "WET%04d.CSV";
const int     MAX_LOG_FILES        = 9999;
const char*   BLE_DEVICE_NAME      = "PHYTOS31 LOGGER";

// ---- Timing ----
const uint32_t SAMPLE_INTERVAL_MS  = 10000;    // one reading (and one CSV row) per sample
const uint32_t STEP_INTERVAL_MS    = 900000;   // the algorithm works on 15-minute averages
const int      SAMPLES_PER_STEP    = STEP_INTERVAL_MS / SAMPLE_INTERVAL_MS;

// ---- ADS1115 ----
// GAIN_TWO = +/-2.048 V range. The PHYTOS31 can output up to about half its
// supply voltage (~1.65 V), so GAIN_FOUR (+/-1.024 V) would clip.
const int       LEAF_ADC_CHANNEL      = 0;
const adsGain_t ADC_GAIN              = GAIN_TWO;
const int       ADC_AVERAGING_SAMPLES = 8;

// ---- Wetting detection (all values in millivolts of sensor output) ----
// Starting values from Binks et al. (2021), who used the same sensor. Their
// units and excitation voltage may differ from yours, so check against your logs.
const float RAIN_RISE_MV       = 100.0f;   // rise in one step that means rain
const float DEW_MIN_RISE_MV    = 1.0f;     // dew: each step rises by at least this...
const float DEW_MAX_RISE_MV    = 15.0f;    // ...and by no more than this...
const int   DEW_MIN_STEPS      = 5;        // ...for this many steps in a row (75 min)

// ---- Ending a wet period ----
const float WET_END_FACTOR     = 1.10f;    // dry again below 110 % of the baseline
const float QUIET_CHANGE_MV    = 0.5f;     // a step this flat counts as "levelled off"
const int   DEW_QUIET_STEPS    = 5;        // dew must also be flat this many steps in a row

// ---- Additions beyond the paper ----
const float RAIN_RESURGE_MV    = 20.0f;    // during rain, a rise this big restarts "time since rain"
const float DRY_FALLBACK_MV    = 350.0f;   // baseline if the logger powers up on an already wet leaf


// =============================================================================
//  Data types
// =============================================================================

enum WetState { STATE_DRY, STATE_DEW, STATE_RAIN, STATE_WET };

struct LeafWetness {
  WetState state          = STATE_DRY;
  float    baselineMv     = 0;       // dry level: before the current wet period began
  float    prevStepMv     = 0;       // previous 15-minute average
  float    lastStepMv     = 0;       // most recent 15-minute average
  float    lastStepRiseMv = 0;       // ...and how much it rose over the one before
  int      dewRunSteps    = 0;       // consecutive dew-sized rises so far
  int      quietSteps     = 0;       // consecutive flat steps so far
  bool     everWet        = false;
  bool     everRained     = false;
  uint32_t wetStartMs     = 0;
  uint32_t dewRunStartMs  = 0;       // when the current run of dew-sized rises began
  uint32_t lastWetEndMs   = 0;
  uint32_t lastRainMs     = 0;       // last time rain was detected or surged
  uint32_t totalWetMs     = 0;       // completed wet periods only
  float    stepSumMv      = 0;       // accumulating the current 15-minute average
  int      stepSamples    = 0;
};

struct AirReading {
  bool  valid;
  float temperatureC;
  float humidityPercent;
  float dewPointC;
};

struct UtcTime {
  bool valid;            // false until the GPS knows the date and time
  int  year, month, day, hour, minute, second;
};

// All values in seconds, ready for logging and display.
struct Durations {
  uint32_t wetFor;       // 0 when dry
  uint32_t dryFor;       // 0 when wet or never wet
  uint32_t sinceRain;    // 0 when no rain seen
  uint32_t totalWet;     // includes the current wet period
};


// =============================================================================
//  Global objects and state
// =============================================================================

Adafruit_ADS1115 adc;                                               // ADS1115
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R2, U8X8_PIN_NONE);   // RAK1921, rotated 180 degrees
Adafruit_BME680 airSensor(&Wire);                                   // RAK1906
SFE_UBLOX_GNSS  gnss;                                               // GPS
BLEDfu          bleDfu;                                             // over-the-air firmware updates
BLEUart         bleUart;                                            // log rows over Bluetooth

LeafWetness wetness;
bool     airSensorReady = false;
bool     gnssReady      = false;
bool     sdCardReady    = false;
char     logFileName[16] = "";      // chosen at startup by chooseNewLogFileName()
uint32_t lastSampleMs   = 0;


// =============================================================================
//  Small helpers
// =============================================================================

uint32_t secondsBetween(uint32_t laterMs, uint32_t earlierMs) {
  return (laterMs - earlierMs) / 1000;      // unsigned math survives millis() rollover
}

// Formats seconds as "45s", "12m 05s", "3h 20m" or "2d 04h".
void formatDuration(uint32_t seconds, char* out, size_t outSize) {
  unsigned long s = seconds;
  if      (s >= 86400) snprintf(out, outSize, "%lud %02luh", s / 86400, (s % 86400) / 3600);
  else if (s >= 3600)  snprintf(out, outSize, "%luh %02lum", s / 3600,  (s % 3600) / 60);
  else if (s >= 60)    snprintf(out, outSize, "%lum %02lus", s / 60,    s % 60);
  else                 snprintf(out, outSize, "%lus", s);
}

// Formats GPS time as "2026-10-08,14:03:09" (the CSV date and time columns),
// or just "," when there is no valid time yet.
void formatTimestamp(const UtcTime& t, char* out, size_t outSize) {
  if (!t.valid) {
    snprintf(out, outSize, ",");
    return;
  }
  snprintf(out, outSize, "%04d-%02d-%02d,%02d:%02d:%02d",
           t.year, t.month, t.day, t.hour, t.minute, t.second);
}

// Dew point from temperature and relative humidity (Magnus formula).
float dewPointC(float temperatureC, float humidityPercent) {
  const float A = 17.62f;
  const float B = 243.12f;
  float g = log(humidityPercent / 100.0f) + A * temperatureC / (B + temperatureC);
  return B * g / (A - g);
}

const char* stateName(WetState state) {
  switch (state) {
    case STATE_DEW:  return "DEW";
    case STATE_RAIN: return "RAIN";
    case STATE_WET:  return "WET";      // wet, but not shaped like dew or heavy rain
    default:         return "DRY";
  }
}

bool isWet(const LeafWetness& w) {
  return w.state != STATE_DRY;
}

// Reports a fatal startup problem forever (the logger is useless without it).
// Shown on the OLED, and on Serial in debug builds.
void haltWithError(const char* message) {
  oled.clearBuffer();
  oled.drawStr(0, 9, message);
  oled.sendBuffer();
  while (true) {
    DEBUG_PRINTLN(message);
    delay(2000);
  }
}


// =============================================================================
//  Sensor reading
// =============================================================================

float readLeafMv() {
  float sum = 0;
  for (int i = 0; i < ADC_AVERAGING_SAMPLES; i++) {
    sum += adc.computeVolts(adc.readADC_SingleEnded(LEAF_ADC_CHANNEL));
  }
  return sum / ADC_AVERAGING_SAMPLES * 1000.0f;
}

AirReading readAir() {
  AirReading air = { false, NAN, NAN, NAN };
  if (!airSensorReady || !airSensor.performReading()) return air;

  air.valid           = true;
  air.temperatureC    = airSensor.temperature;
  air.humidityPercent = airSensor.humidity;
  air.dewPointC       = dewPointC(air.temperatureC, air.humidityPercent);
  return air;
}

UtcTime readUtcTime() {
  UtcTime t = { false, 0, 0, 0, 0, 0, 0 };
  if (!gnssReady || !gnss.getDateValid() || !gnss.getTimeValid()) return t;

  t.valid  = true;
  t.year   = gnss.getYear();
  t.month  = gnss.getMonth();
  t.day    = gnss.getDay();
  t.hour   = gnss.getHour();
  t.minute = gnss.getMinute();
  t.second = gnss.getSecond();
  return t;
}


// =============================================================================
//  Wetness tracking (Binks-style, on 15-minute steps)
// =============================================================================

void markRain(LeafWetness& w, uint32_t nowMs) {
  w.everRained = true;
  w.lastRainMs = nowMs;
}

// Begins a wet period. The rise that triggered it happened sometime within the
// last step (or, for dew, started at the beginning of the run), so startMs is
// back-dated to there.
void startWetPeriod(LeafWetness& w, WetState newState, uint32_t startMs, uint32_t nowMs) {
  w.state       = newState;
  w.everWet     = true;
  w.wetStartMs  = startMs;
  w.dewRunSteps = 0;
  if (newState == STATE_RAIN) markRain(w, nowMs);
}

void endWetPeriod(LeafWetness& w, float stepMv, uint32_t nowMs) {
  w.totalWetMs   += nowMs - w.wetStartMs;
  w.lastWetEndMs  = nowMs;
  w.state         = STATE_DRY;
  w.baselineMv    = stepMv;
  w.dewRunSteps   = 0;
}

// Dry leaf: watch for the signal starting to climb.
void detectWetting(LeafWetness& w, float riseMv, uint32_t nowMs) {
  uint32_t stepStartMs = nowMs - STEP_INTERVAL_MS;

  // Follow the dry signal (drift, dirt) until a rise begins, then freeze it.
  if (w.dewRunSteps == 0) w.baselineMv = w.prevStepMv;

  if (riseMv >= RAIN_RISE_MV) {
    startWetPeriod(w, STATE_RAIN, stepStartMs, nowMs);
  } else if (riseMv > DEW_MAX_RISE_MV) {
    startWetPeriod(w, STATE_WET, stepStartMs, nowMs);
  } else if (riseMv >= DEW_MIN_RISE_MV) {
    if (w.dewRunSteps == 0) w.dewRunStartMs = stepStartMs;
    w.dewRunSteps++;
    if (w.dewRunSteps >= DEW_MIN_STEPS) startWetPeriod(w, STATE_DEW, w.dewRunStartMs, nowMs);
  } else {
    w.dewRunSteps = 0;
  }
}

// Wet leaf: watch for rain landing on top, and for the signal returning to baseline.
void trackWetPeriod(LeafWetness& w, float stepMv, float riseMv, uint32_t nowMs) {
  if (riseMv >= RAIN_RISE_MV) {
    w.state = STATE_RAIN;             // rain on a dewy or damp leaf; keep the old baseline
    markRain(w, nowMs);
  } else if (w.state == STATE_RAIN && riseMv >= RAIN_RESURGE_MV) {
    markRain(w, nowMs);               // more water arriving during rain
  }

  bool backAtBaseline = stepMv < w.baselineMv * WET_END_FACTOR;
  bool levelledOff    = w.state != STATE_DEW || w.quietSteps >= DEW_QUIET_STEPS;
  if (backAtBaseline && levelledOff) endWetPeriod(w, stepMv, nowMs);
}

// Handles one finished 15-minute average.
void processStep(LeafWetness& w, float stepMv, uint32_t nowMs) {
  float riseMv = stepMv - w.prevStepMv;

  w.lastStepMv     = stepMv;
  w.lastStepRiseMv = riseMv;
  w.quietSteps     = (fabs(riseMv) < QUIET_CHANGE_MV) ? w.quietSteps + 1 : 0;

  if (isWet(w)) trackWetPeriod(w, stepMv, riseMv, nowMs);
  else          detectWetting(w, riseMv, nowMs);

  w.prevStepMv = stepMv;
}

// Adds one reading to the running 15-minute average.
void addSample(LeafWetness& w, float leafMv, uint32_t nowMs) {
  w.stepSumMv += leafMv;
  w.stepSamples++;
  if (w.stepSamples < SAMPLES_PER_STEP) return;

  float stepMv = w.stepSumMv / w.stepSamples;
  w.stepSumMv   = 0;
  w.stepSamples = 0;
  processStep(w, stepMv, nowMs);
}

// First reading after power-up: use it as the baseline, unless the leaf is
// already clearly wet, in which case fall back to a typical dry level.
void initialiseWetness(LeafWetness& w, float firstMv, uint32_t nowMs) {
  w.baselineMv = firstMv;
  w.prevStepMv = firstMv;
  w.lastStepMv = firstMv;

  if (firstMv > DRY_FALLBACK_MV * WET_END_FACTOR) {
    w.baselineMv = DRY_FALLBACK_MV;
    startWetPeriod(w, STATE_WET, nowMs, nowMs);
  }
}

Durations computeDurations(const LeafWetness& w, uint32_t nowMs) {
  Durations d;
  d.wetFor    = isWet(w) ? secondsBetween(nowMs, w.wetStartMs) : 0;
  d.dryFor    = (!isWet(w) && w.everWet) ? secondsBetween(nowMs, w.lastWetEndMs) : 0;
  d.sinceRain = w.everRained ? secondsBetween(nowMs, w.lastRainMs) : 0;

  uint32_t currentWetMs = isWet(w) ? (nowMs - w.wetStartMs) : 0;
  d.totalWet  = (w.totalWetMs + currentWetMs) / 1000;
  return d;
}


// =============================================================================
//  Logging (Serial + SD card + Bluetooth)
// =============================================================================

// mv = leaf reading now; state, baseline_mv, step_mv and step_rise_mv only
// change when a 15-minute step finishes.
const char* CSV_HEADER =
  "date,time,uptime_s,mv,state,baseline_mv,step_mv,step_rise_mv,wet_dur_s,"
  "since_wet_s,since_rain_s,wet_total_s,temp_c,rh_pct,dewpt_c";

void formatCsvRow(char* out, size_t outSize, uint32_t nowMs, const UtcTime& utc,
                  float leafMv, const LeafWetness& w, const AirReading& air,
                  const Durations& d) {
  char timestamp[24];
  formatTimestamp(utc, timestamp, sizeof timestamp);

  snprintf(out, outSize,
           "%s,%lu,%.1f,%s,%.1f,%.1f,%.1f,%lu,%lu,%lu,%lu,%.1f,%.1f,%.1f",
           timestamp, (unsigned long)(nowMs / 1000),
           leafMv, stateName(w.state), w.baselineMv, w.lastStepMv, w.lastStepRiseMv,
           (unsigned long)d.wetFor, (unsigned long)d.dryFor,
           (unsigned long)d.sinceRain, (unsigned long)d.totalWet,
           air.temperatureC, air.humidityPercent, air.dewPointC);
}

// Opens and closes the file on every write so a power cut can't corrupt it.
bool appendToLogFile(const char* line) {
  File file = SD.open(logFileName, FILE_WRITE);   // FILE_WRITE appends
  if (!file) return false;
  file.println(line);
  file.close();
  return true;
}

void logLine(const char* line) {
  DEBUG_PRINTLN(line);

  if (bleUart.notifyEnabled()) bleUart.println(line);

  if (sdCardReady && !appendToLogFile(line)) {
    sdCardReady = false;
    DEBUG_PRINTLN("# SD write failed");
  }
}


// =============================================================================
//  Display (6 text rows, 10 px apart)
// =============================================================================

void drawRow(int row, const char* text) {
  const int FIRST_BASELINE_Y = 9;
  const int ROW_HEIGHT       = 10;
  oled.drawStr(0, FIRST_BASELINE_Y + row * ROW_HEIGHT, text);
}

void drawDisplay(float leafMv, const AirReading& air, const UtcTime& utc,
                 const LeafWetness& w, const Durations& d) {
  char text[24];
  char duration[12];      // longest is "49710d 23h"

  oled.clearBuffer();

  // Row 0: wet/dry state, warnings for missing SD card or GPS time
  snprintf(text, sizeof text, "%s%s%s",
           stateName(w.state),
           sdCardReady ? "" : " !SD",
           utc.valid   ? "" : " !GPS");
  drawRow(0, text);

  // Row 1: leaf reading and baseline
  snprintf(text, sizeof text, "%.0f mV base %.0f", leafMv, w.baselineMv);
  drawRow(1, text);

  // Row 2: air conditions
  if (air.valid) {
    snprintf(text, sizeof text, "%.1fC %.0f%% dp%.1f",
             air.temperatureC, air.humidityPercent, air.dewPointC);
  } else {
    snprintf(text, sizeof text, "No air sensor");
  }
  drawRow(2, text);

  // Row 3: how long it has been wet or dry
  if (isWet(w)) {
    formatDuration(d.wetFor, duration, sizeof duration);
    snprintf(text, sizeof text, "Wet for %s", duration);
  } else if (w.everWet) {
    formatDuration(d.dryFor, duration, sizeof duration);
    snprintf(text, sizeof text, "Dry for %s", duration);
  } else {
    snprintf(text, sizeof text, "Dry since boot");
  }
  drawRow(3, text);

  // Row 4: time since rain was last detected
  if (!w.everRained) {
    snprintf(text, sizeof text, "Rain: none seen");
  } else {
    formatDuration(d.sinceRain, duration, sizeof duration);
    snprintf(text, sizeof text, "Rain: %s ago", duration);
  }
  drawRow(4, text);

  // Row 5: cumulative wet time
  formatDuration(d.totalWet, duration, sizeof duration);
  snprintf(text, sizeof text, "Wet total %s", duration);
  drawRow(5, text);

  oled.sendBuffer();
}


// =============================================================================
//  Bluetooth callbacks
// =============================================================================

#if DEBUG   // these callbacks only report to Serial
void onBleConnect(uint16_t connHandle) {
  (void)connHandle;
  Serial.println("# BLE connected");
}

void onBleDisconnect(uint16_t connHandle, uint8_t reason) {
  (void)connHandle;
  (void)reason;
  Serial.println("# BLE disconnected");
}
#endif


// =============================================================================
//  Setup
// =============================================================================

#if DEBUG
void startDebugSerial() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) delay(10);   // wait up to 3 s for a monitor
}
#endif

void startSensorPower() {
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);   // kept on: the ADC and OLED share this rail
  delay(300);
}

void startAdc() {
  if (!adc.begin()) haltWithError("# ADS1115 not found");
  adc.setGain(ADC_GAIN);
}

void startDisplay() {
  oled.begin();
  oled.setFont(u8g2_font_6x10_tf);
}

void startAirSensor() {
  airSensorReady = airSensor.begin(BME680_I2C_ADDRESS);
  if (!airSensorReady) {
    DEBUG_PRINTLN("# RAK1906 not found");
    return;
  }
  airSensor.setTemperatureOversampling(BME680_OS_8X);
  airSensor.setHumidityOversampling(BME680_OS_2X);
  airSensor.setGasHeater(0, 0);           // heater off: not needed, and avoids self-heating
}

void startGnss() {
  gnssReady = gnss.begin();
  if (!gnssReady) {
    DEBUG_PRINTLN("# GPS not found");
    return;
  }
  gnss.setI2COutput(COM_TYPE_UBX);                    // I2C: UBX messages only
  gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);    // remember that setting
}

// Finds the first unused file name so every boot gets a fresh file and earlier
// logs are never touched.
bool chooseNewLogFileName() {
  for (int n = 1; n <= MAX_LOG_FILES; n++) {
    snprintf(logFileName, sizeof logFileName, LOG_FILE_PATTERN, n);
    if (!SD.exists(logFileName)) return true;
  }
  return false;
}

void startSdCard() {
  sdCardReady = SD.begin();
  if (!sdCardReady) {
    DEBUG_PRINTLN("# SD card not found");
    return;
  }
  if (!chooseNewLogFileName()) {
    sdCardReady = false;
    DEBUG_PRINTLN("# No free log file name on SD card");
    return;
  }
  appendToLogFile(CSV_HEADER);
  DEBUG_PRINT("# Logging to ");
  DEBUG_PRINTLN(logFileName);
}

void startBluetooth() {
  // Bandwidth and connection settings must be set before Bluefruit.begin().
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configPrphConn(92, BLE_GAP_EVENT_LENGTH_MIN, 16, 16);

  if (!Bluefruit.begin()) {
    DEBUG_PRINTLN("# Bluetooth failed to start");
    return;
  }
  Bluefruit.setTxPower(8);
  Bluefruit.setName(BLE_DEVICE_NAME);
#if DEBUG
  Bluefruit.Periph.setConnectCallback(onBleConnect);
  Bluefruit.Periph.setDisconnectCallback(onBleDisconnect);
#endif

  bleDfu.begin();
  bleUart.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleUart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);                     // advertise forever
}

void setup() {
#if DEBUG
  startDebugSerial();
#endif

  startSensorPower();
  Wire.begin();
  startDisplay();          // before the ADC so a fatal ADC error can be shown on screen
  startAdc();
  startAirSensor();
  startGnss();
  startSdCard();
  startBluetooth();
  initialiseWetness(wetness, readLeafMv(), millis());

  DEBUG_PRINTLN(CSV_HEADER);
}


// =============================================================================
//  Main loop
// =============================================================================

void loop() {
  uint32_t nowMs = millis();
  if (nowMs - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = nowMs;

  float      leafMv = readLeafMv();
  AirReading air    = readAir();
  UtcTime    utc    = readUtcTime();

  addSample(wetness, leafMv, nowMs);
  Durations durations = computeDurations(wetness, nowMs);

  char row[200];
  formatCsvRow(row, sizeof row, nowMs, utc, leafMv, wetness, air, durations);
  logLine(row);

  drawDisplay(leafMv, air, utc, wetness, durations);
}
