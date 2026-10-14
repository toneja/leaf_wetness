/*
  Leaf wetness logger
  -------------------
  Hardware : RAK4631 core, PHYTOS31 leaf wetness sensor read through an
             Adafruit ADS1115 16-bit ADC, RAK1906 (BME680), RAK1921 OLED,
             RAK15002 microSD, u-blox GNSS module (I2C)
  Libraries: Adafruit ADS1X15, U8g2, Adafruit BME680 (+ Adafruit Unified Sensor),
             SparkFun u-blox GNSS; SD.h and bluefruit.h are built in

  Wiring:
    PHYTOS31  supply -> 3V3_S,  ground -> GND,  signal -> ADS1115 A0
    ADS1115   VDD -> 3V3_S,  GND -> GND,  SDA/SCL -> the WisBlock I2C bus (address 0x48)

  What it does, every SAMPLE_INTERVAL_MS:
    1. Reads leaf wetness (volts -> percent), air temperature / humidity, and GPS time.
    2. Decides whether the leaf is wet, and why (rain, dew, or unknown).
    3. Tracks how long it has been wet, dry, and since the last rain.
    4. Sends one CSV row to the SD card and Bluetooth (and to Serial when DEBUG is 1),
       and updates the OLED.

  GPS time is only used to timestamp rows. The wet/dry durations come from the
  internal clock, so logging works (with blank date/time) before the GPS has a fix.
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
const uint32_t SAMPLE_INTERVAL_MS  = 10000;
const uint32_t RISE_WINDOW_MS      = 60000;    // "rise" = change over this window
const int      RISE_WINDOW_SAMPLES = RISE_WINDOW_MS / SAMPLE_INTERVAL_MS;

// ---- ADS1115 ----
// GAIN_TWO = +/-2.048 V range (62.5 uV per bit). The PHYTOS31 can output up to
// about half its supply voltage (~1.65 V), so GAIN_FOUR (+/-1.024 V) would clip.
const int       LEAF_ADC_CHANNEL      = 0;
const adsGain_t ADC_GAIN              = GAIN_TWO;
const int       ADC_AVERAGING_SAMPLES = 8;

// ---- Leaf wetness calibration ----
// Output voltage with a clean dry sensor and a fully wet one. The range differs
// a little for every sensor, so re-measure these for yours. The output scales
// with supply voltage, so keep the supply steady (3V3_S).
const float LEAF_DRY_VOLTS         = 0.350f;
const float LEAF_WET_VOLTS         = 1.100f;

// ---- Leaf wetness thresholds (percent of the dry-to-wet range) ----
const float WET_ON_PERCENT         = 3.0f;     // at or above: leaf becomes wet
const float WET_OFF_PERCENT        = 1.5f;     // at or below: leaf becomes dry
const float FILM_UPPER_PERCENT     = 20.0f;    // below this: thin film / dew
const float WET_UPPER_PERCENT      = 75.0f;    // below this: wet; above: drenched

// ---- Rain detection ----
// Rain shows up two ways in real logs: a fast climb when the leaf first gets
// soaked, and then constant jitter as drops land and run off (steps of 0.5 % or
// more every few samples). A dry or merely damp sensor changes by 0.2 % or less
// per sample, so counting big steps separates rain from dew and slow drying.
const float    RAIN_MIN_RISE_PERCENT = 5.0f;   // rise within RISE_WINDOW that means rain
const float    RAIN_STEP_PERCENT     = 0.5f;   // one sample-to-sample change this big is a "drop"
const int      RAIN_MIN_BIG_STEPS    = 3;      // this many drops within RISE_WINDOW means rain
const uint32_t RAIN_HOLD_MS          = 300000; // rain counts as ongoing this long after the last drop

// ---- Wet-source detection ----
const float DEW_MAX_DEPRESSION_C   = 1.5f;     // air this close to dew point => dew likely
const float FROST_BELOW_C          = 1.0f;     // ice can read as "dry" below this


// =============================================================================
//  Data types
// =============================================================================

struct LeafReading {
  float volts;
  float percent;         // 0 = dry calibration point, 100 = wet calibration point
  float risePercent;     // change since RISE_WINDOW_MS ago
  int   bigSteps;        // sample-to-sample jumps of RAIN_STEP_PERCENT or more within RISE_WINDOW_MS
};

struct AirReading {
  bool  valid;
  float temperatureC;
  float humidityPercent;
  float dewPointC;
  float dewDepressionC;  // temperature minus dew point
};

struct UtcTime {
  bool valid;            // false until the GPS knows the date and time
  int  year, month, day, hour, minute, second;
};

enum WetSource { SOURCE_NONE, SOURCE_RAIN, SOURCE_DEW, SOURCE_UNKNOWN };

struct WetnessState {
  bool      isWet         = false;
  bool      rainOngoing   = false;   // rain activity seen within RAIN_HOLD_MS
  bool      everWet       = false;
  bool      everRained    = false;
  WetSource source        = SOURCE_NONE;
  uint32_t  wetStartMs    = 0;
  uint32_t  lastDryMs     = 0;
  uint32_t  lastRainMs    = 0;
  uint32_t  totalWetMs    = 0;       // completed wet periods only
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

WetnessState wetness;
bool     airSensorReady = false;
bool     gnssReady      = false;
bool     sdCardReady    = false;
char     logFileName[16] = "";      // chosen at startup by chooseNewLogFileName()
uint32_t lastSampleMs   = 0;

// Ring buffer of recent leaf readings, used to measure how fast wetness rises.
float recentPercent[RISE_WINDOW_SAMPLES];
int   oldestReadingIndex = 0;


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

const char* wetSourceName(WetSource source) {
  switch (source) {
    case SOURCE_RAIN:    return "RAIN";
    case SOURCE_DEW:     return "DEW";
    case SOURCE_UNKNOWN: return "WET";
    default:             return "-";
  }
}

const char* wetnessLevelName(float percent) {
  if (percent < WET_OFF_PERCENT)    return "DRY";
  if (percent < FILM_UPPER_PERCENT) return "FILM";
  if (percent < WET_UPPER_PERCENT)  return "WET";
  return "DRENCHED";
}

bool isFrostRisk(const AirReading& air) {
  return air.valid && air.temperatureC < FROST_BELOW_C;
}

float voltsToPercent(float volts) {
  return (volts - LEAF_DRY_VOLTS) / (LEAF_WET_VOLTS - LEAF_DRY_VOLTS) * 100.0f;
}

// Shows a fatal startup problem on the OLED (and Serial in debug) and stops,
// because the logger is useless without it.
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

float readLeafVolts() {
  float sum = 0;
  for (int i = 0; i < ADC_AVERAGING_SAMPLES; i++) {
    sum += adc.computeVolts(adc.readADC_SingleEnded(LEAF_ADC_CHANNEL));
  }
  return sum / ADC_AVERAGING_SAMPLES;
}

// Stores the new reading and returns how much it differs from RISE_WINDOW_MS ago.
float recordReadingAndGetRise(float percent) {
  float rise = percent - recentPercent[oldestReadingIndex];
  recentPercent[oldestReadingIndex] = percent;
  oldestReadingIndex = (oldestReadingIndex + 1) % RISE_WINDOW_SAMPLES;
  return rise;
}

// Counts how many sample-to-sample changes in the history were big (drops landing).
// Call after recordReadingAndGetRise(): the oldest reading is then at oldestReadingIndex.
int countBigSteps() {
  int count = 0;
  for (int i = 0; i < RISE_WINDOW_SAMPLES - 1; i++) {
    float older = recentPercent[(oldestReadingIndex + i)     % RISE_WINDOW_SAMPLES];
    float newer = recentPercent[(oldestReadingIndex + i + 1) % RISE_WINDOW_SAMPLES];
    if (fabs(newer - older) >= RAIN_STEP_PERCENT) count++;
  }
  return count;
}

LeafReading readLeaf() {
  LeafReading leaf;
  leaf.volts       = readLeafVolts();
  leaf.percent     = voltsToPercent(leaf.volts);
  leaf.risePercent = recordReadingAndGetRise(leaf.percent);
  leaf.bigSteps    = countBigSteps();
  return leaf;
}

AirReading readAir() {
  AirReading air = { false, NAN, NAN, NAN, NAN };
  if (!airSensorReady || !airSensor.performReading()) return air;

  air.valid           = true;
  air.temperatureC    = airSensor.temperature;
  air.humidityPercent = airSensor.humidity;
  air.dewPointC       = dewPointC(air.temperatureC, air.humidityPercent);
  air.dewDepressionC  = air.temperatureC - air.dewPointC;
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
//  Wetness tracking
// =============================================================================

// Rain = a fast climb, or a burst of drops landing. Slow dew or drying never qualifies.
bool looksLikeRain(const LeafReading& leaf) {
  return leaf.risePercent >= RAIN_MIN_RISE_PERCENT || leaf.bigSteps >= RAIN_MIN_BIG_STEPS;
}

// Guesses why the leaf became wet, using how fast it rose and how close the air
// is to its dew point.
WetSource guessWetSource(const LeafReading& leaf, const AirReading& air) {
  if (looksLikeRain(leaf))                                        return SOURCE_RAIN;
  if (air.valid && air.dewDepressionC < DEW_MAX_DEPRESSION_C)     return SOURCE_DEW;
  return SOURCE_UNKNOWN;
}

void startWetPeriod(WetnessState& w, const LeafReading& leaf, const AirReading& air, uint32_t nowMs) {
  w.isWet      = true;
  w.everWet    = true;
  w.wetStartMs = nowMs;
  w.source     = guessWetSource(leaf, air);
}

void endWetPeriod(WetnessState& w, uint32_t nowMs) {
  w.isWet       = false;
  w.source      = SOURCE_NONE;
  w.lastDryMs   = nowMs;
  w.totalWetMs += nowMs - w.wetStartMs;
}

void updateWetness(WetnessState& w, const LeafReading& leaf, const AirReading& air, uint32_t nowMs) {
  // 1. Wet/dry transitions, with hysteresis so noise can't cause flicker.
  if (!w.isWet && leaf.percent >= WET_ON_PERCENT) {
    startWetPeriod(w, leaf, air, nowMs);
  } else if (w.isWet && leaf.percent <= WET_OFF_PERCENT) {
    endWetPeriod(w, nowMs);
  }

  // 2. Rain (or irrigation) activity while wet, even on an already wet leaf.
  //    Every sample with activity restarts the "time since rain" clock.
  if (w.isWet && looksLikeRain(leaf)) {
    w.everRained = true;
    w.lastRainMs = nowMs;
    w.source     = SOURCE_RAIN;
  }

  // 3. Rain stays "ongoing" for a few minutes after the last drop, so gaps between
  //    drops don't flicker it off. Dry spells then count up from the last drop.
  w.rainOngoing = w.everRained && (nowMs - w.lastRainMs) < RAIN_HOLD_MS;
}

Durations computeDurations(const WetnessState& w, uint32_t nowMs) {
  Durations d;
  d.wetFor    = w.isWet ? secondsBetween(nowMs, w.wetStartMs) : 0;
  d.dryFor    = (!w.isWet && w.everWet) ? secondsBetween(nowMs, w.lastDryMs) : 0;
  d.sinceRain = w.everRained ? secondsBetween(nowMs, w.lastRainMs) : 0;

  uint32_t currentWetMs = w.isWet ? (nowMs - w.wetStartMs) : 0;
  d.totalWet  = (w.totalWetMs + currentWetMs) / 1000;
  return d;
}


// =============================================================================
//  Logging (Serial + SD card + Bluetooth)
// =============================================================================

const char* CSV_HEADER =
  "date,time,uptime_s,volts,wet_pct,class,rise_pct_min,wet,source,wet_dur_s,"
  "since_wet_s,since_rain_s,wet_total_s,temp_c,rh_pct,dewpt_c,depr_c,frost";

void formatCsvRow(char* out, size_t outSize, uint32_t nowMs, const UtcTime& utc,
                  const LeafReading& leaf, const AirReading& air,
                  const WetnessState& w, const Durations& d) {
  char timestamp[24];
  formatTimestamp(utc, timestamp, sizeof timestamp);

  snprintf(out, outSize,
           "%s,%lu,%.4f,%.1f,%s,%.1f,%d,%s,%lu,%lu,%lu,%lu,%.1f,%.1f,%.1f,%.1f,%d",
           timestamp, (unsigned long)(nowMs / 1000),
           leaf.volts, leaf.percent, wetnessLevelName(leaf.percent), leaf.risePercent,
           w.isWet ? 1 : 0, wetSourceName(w.source),
           (unsigned long)d.wetFor, (unsigned long)d.dryFor,
           (unsigned long)d.sinceRain, (unsigned long)d.totalWet,
           air.temperatureC, air.humidityPercent, air.dewPointC, air.dewDepressionC,
           isFrostRisk(air) ? 1 : 0);
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

void drawDisplay(const LeafReading& leaf, const AirReading& air, const UtcTime& utc,
                 const WetnessState& w, const Durations& d) {
  char text[24];
  char duration[24];

  oled.clearBuffer();

  // Row 0: wet/dry state, source, warnings for missing SD card or GPS time
  snprintf(text, sizeof text, "%s %s%s%s",
           w.isWet ? "WET" : "dry",
           w.isWet ? wetSourceName(w.source) : "",
           sdCardReady ? "" : " !SD",
           utc.valid   ? "" : " !GPS");
  drawRow(0, text);

  // Row 1: leaf voltage, percent and level
  snprintf(text, sizeof text, "%.3fV %.0f%% %s",
           leaf.volts, leaf.percent, wetnessLevelName(leaf.percent));
  drawRow(1, text);

  // Row 2: air conditions (dd = dew point depression)
  if (air.valid) {
    snprintf(text, sizeof text, "%.1fC %.0f%% dd%.1f%s",
             air.temperatureC, air.humidityPercent, air.dewDepressionC,
             isFrostRisk(air) ? " ICE?" : "");
  } else {
    snprintf(text, sizeof text, "No air sensor");
  }
  drawRow(2, text);

  // Row 3: how long it has been wet or dry
  if (w.isWet) {
    formatDuration(d.wetFor, duration, sizeof duration);
    snprintf(text, sizeof text, "Wet for %s", duration);
  } else if (w.everWet) {
    formatDuration(d.dryFor, duration, sizeof duration);
    snprintf(text, sizeof text, "Dry for %s", duration);
  } else {
    snprintf(text, sizeof text, "Dry since boot");
  }
  drawRow(3, text);

  // Row 4: time since rain
  if (!w.everRained) {
    snprintf(text, sizeof text, "Rain: none seen");
  } else if (w.rainOngoing) {
    snprintf(text, sizeof text, "Rain: ongoing");
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

// Fills the history with the first reading so the first "rise" is zero, and
// handles the case where the leaf is already wet at power-on.
void initialiseWetnessFromFirstReading() {
  float firstPercent = voltsToPercent(readLeafVolts());
  for (int i = 0; i < RISE_WINDOW_SAMPLES; i++) recentPercent[i] = firstPercent;

  if (firstPercent >= WET_ON_PERCENT) {
    wetness.isWet      = true;
    wetness.everWet    = true;
    wetness.wetStartMs = millis();
    wetness.source     = SOURCE_UNKNOWN;
  }
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
  initialiseWetnessFromFirstReading();

  DEBUG_PRINTLN(CSV_HEADER);
}


// =============================================================================
//  Main loop
// =============================================================================

void loop() {
  uint32_t nowMs = millis();
  if (nowMs - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = nowMs;

  LeafReading leaf = readLeaf();
  AirReading  air  = readAir();
  UtcTime     utc  = readUtcTime();

  updateWetness(wetness, leaf, air, nowMs);
  Durations durations = computeDurations(wetness, nowMs);

  char row[220];
  formatCsvRow(row, sizeof row, nowMs, utc, leaf, air, wetness, durations);
  logLine(row);

  drawDisplay(leaf, air, utc, wetness, durations);
}
