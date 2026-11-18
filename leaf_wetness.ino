/*
  Leaf wetness logger, v9
  -----------------------
  Hardware : RAK4631 core, PHYTOS31 leaf wetness sensor read through an
             Adafruit ADS1115 16-bit ADC, RAK1906 (BME680), RAK1921 OLED,
             RAK15002 microSD, u-blox GNSS module (I2C),
             Texas Electronics TR-525M tipping bucket rain gauge
  Libraries: Adafruit ADS1X15, U8g2, Adafruit BME680 (+ Adafruit Unified Sensor),
             SparkFun u-blox GNSS; SD.h and bluefruit.h are built in

  Wiring:
    PHYTOS31  supply -> 3V3_S,  ground -> GND,  signal -> ADS1115 A0
    ADS1115   VDD -> 3V3_S,  GND -> GND,  SDA/SCL -> the WisBlock I2C bus (address 0x48)
    TR-525M   one reed switch lead -> WB_IO3,  the other -> GND  (polarity does not matter)

  How the leaf is judged (two separate questions, both answered every 10 s):

    "Is the leaf wet?"   Level test. The sensor reading is compared with a dry
                         baseline that the sketch learns for itself. Wet when it
                         is WET_ON_MV above the baseline, dry again below
                         WET_OFF_MV. The baseline creeps along with slow drift
                         while the leaf is dry and is frozen while it is wet.

    "Is water arriving?" Rate test. If the reading has risen by RAIN_RISE_MV or
                         more over the last 10 minutes, or the rain gauge tipped,
                         that counts as rain. Dew and temperature drift rise far
                         more slowly than that. Rain stays "active" for
                         RAIN_HOLD_MS after the last such signal.

  Resulting states:
    DRY   leaf is at its baseline
    RAIN  wet, and rain was detected within the last RAIN_HOLD_MS
    WET   wet, rain was detected earlier in this wet period but not recently
    DEW   wet, and no rain at all has been detected during this wet period

  This replaces the 15-minute-average method of v6/v7. Light rain on a tilted
  sensor raises the signal by only a few millivolts in small steps, which
  averaging over 15 minutes hides. See the README for what the logs showed.

  Every SAMPLE_INTERVAL_MS the sketch reads the sensor and the air sensor, and
  sends one CSV row to the SD card (and Serial when DEBUG is 1), and updates
  the OLED.

  Bluetooth: the logger is a BLE central. It scans for devices that offer the
  Nordic UART service, connects to up to MAX_PERIPHERALS of them, and after every
  sample sends each one a single line:

      raining,rain_24h_mm,seconds_since_rain_stopped      e.g.  0,2.5,7200

    raining                1 while the state is RAIN, otherwise 0
    rain_24h_mm            gauge rainfall over the last 24 hours (since power-on
                           if the logger has run for less than that)
    seconds_since_rain_stopped
                           0 while raining, counting up once RAIN ends;
                           -1 if no rain has been seen since power-on

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
const int     MAX_PERIPHERALS      = 4;        // BLE devices the logger will stay connected to
const int     RAIN_GAUGE_PIN       = WB_IO3;   // TR-525M reed switch to GND

// ---- Timing ----
const uint32_t SAMPLE_INTERVAL_MS  = 10000;    // one reading (and one CSV row) per sample
// ---- ADS1115 ----
// GAIN_TWO = +/-2.048 V range. The PHYTOS31 can output up to about half its
// supply voltage (~1.65 V), so GAIN_FOUR (+/-1.024 V) would clip.
const int       LEAF_ADC_CHANNEL      = 0;
const adsGain_t ADC_GAIN              = GAIN_TWO;
const int       ADC_AVERAGING_SAMPLES = 8;

// ---- Rain gauge ----
const float    RAIN_GAUGE_MM_PER_TIP = 0.1f;   // TR-525M: 0.1 mm (0.004 in) per tip
const int      RAIN_HISTORY_HOURS   = 24;     // window for the rain_24h_mm value
const uint32_t TIP_DEBOUNCE_MS       = 50;     // ignore closures this soon after a counted tip
                                               // (the switch stays closed ~135 ms per tip)

// ---- Is the leaf wet? (level test, millivolts above the dry baseline) ----
const float WET_ON_MV          = 20.0f;    // wet once this far above baseline
const float WET_OFF_MV         = 15.0f;    // dry again below this (hysteresis against flicker)
const float WET_START_MV       = 3.0f;     // wet period is back-dated to when the signal was last this close
const float BASELINE_FOLLOW_MV = 8.0f;     // while dry, drift smaller than this is followed...
const float BASELINE_SMOOTHING = 0.05f;    // ...slowly (fraction per sample)
const float DRY_FALLBACK_MV    = 350.0f;   // baseline if the logger powers up on an already wet leaf

// ---- Is water arriving? (rate test) ----
const int      RISE_WINDOW_SAMPLES = 60;                 // 60 x 10 s = 10 minutes
const float    RAIN_RISE_MV        = 4.0f;               // rise within the window that counts as rain
const uint32_t RAIN_HOLD_MS        = 30UL * 60 * 1000;   // rain stays "active" this long after the last signal


// =============================================================================
//  Data types
// =============================================================================

enum WetState { STATE_DRY, STATE_DEW, STATE_RAIN, STATE_WET };

struct LeafWetness {
  WetState state            = STATE_DRY;
  float    baselineMv       = 0;       // dry level
  float    riseMv           = 0;       // change over the last RISE_WINDOW_SAMPLES samples
  uint8_t  rainSignal       = 0;       // this sample: bit 0 = leaf rise, bit 1 = gauge tip
  bool     wet              = false;
  bool     everWet          = false;
  bool     everRained       = false;
  bool     rainThisWet      = false;   // rain seen during the current wet period
  uint32_t wetStartMs       = 0;
  uint32_t lastNearBaseMs   = 0;       // last time the signal was within WET_START_MV of baseline
  uint32_t lastWetEndMs     = 0;
  uint32_t lastRainMs       = 0;       // last rain signal from the leaf or the gauge
  uint32_t totalWetMs       = 0;       // completed wet periods only
  float    history[RISE_WINDOW_SAMPLES];   // ring buffer of recent readings
  int      historyNext      = 0;       // slot that holds the oldest reading
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

struct RainGauge {
  uint32_t tipsNow;      // tips since the previous sample
  uint32_t tipsTotal;    // tips since power-on
  bool     everTipped;
  uint32_t lastTipMs;    // millis() of the most recent tip
};

// Gauge tips counted in one-hour slots, for a rolling 24-hour total.
struct RainHistory {
  uint16_t tipsPerSlot[RAIN_HISTORY_HOURS];
  int      currentSlot;
  uint32_t slotStartMs;
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
BLEClientUart   peripheralUart[MAX_PERIPHERALS];                    // one per connected peripheral

LeafWetness wetness;
RainHistory rainHistory;          // zero-initialised: empty history, first slot starts at power-on

// Written by the rain gauge interrupt, read with interrupts off (see readRainGauge).
volatile uint32_t tipCount     = 0;
volatile uint32_t lastTipTimeMs = 0;
uint32_t tipsAtLastSample      = 0;
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
  return w.wet;
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

// Runs on every closure of the rain gauge switch. Keep it short: no Serial here.
void onRainGaugeTip() {
  uint32_t nowMs = millis();
  bool isBounce  = tipCount > 0 && (nowMs - lastTipTimeMs) < TIP_DEBOUNCE_MS;
  if (isBounce) return;

  tipCount++;
  lastTipTimeMs = nowMs;
}

RainGauge readRainGauge() {
  noInterrupts();                       // so a tip can't land between the two reads
  uint32_t total = tipCount;
  uint32_t last  = lastTipTimeMs;
  interrupts();

  RainGauge g;
  g.tipsTotal  = total;
  g.tipsNow    = total - tipsAtLastSample;
  g.everTipped = total > 0;
  g.lastTipMs  = last;
  tipsAtLastSample = total;
  return g;
}

float rainfallMm(const RainGauge& g) {
  return g.tipsTotal * RAIN_GAUGE_MM_PER_TIP;
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

void markRain(LeafWetness& w, uint32_t nowMs) {
  w.everRained  = true;
  w.rainThisWet = true;
  w.lastRainMs  = nowMs;
}

// The wet period is back-dated to when the signal began to leave the baseline,
// which is earlier than when it crossed WET_ON_MV.
void startWetPeriod(LeafWetness& w, uint32_t startMs) {
  w.wet         = true;
  w.everWet     = true;
  w.wetStartMs  = startMs;
  w.rainThisWet = false;
}

void endWetPeriod(LeafWetness& w, float leafMv, uint32_t nowMs) {
  w.totalWetMs  += nowMs - w.wetStartMs;
  w.lastWetEndMs = nowMs;
  w.wet          = false;
  w.baselineMv   = leafMv;        // re-learn the dry level from where it settled
}

// Stores the reading and returns how much it changed over the window.
float updateRise(LeafWetness& w, float leafMv) {
  float oldest = w.history[w.historyNext];
  w.history[w.historyNext] = leafMv;
  w.historyNext = (w.historyNext + 1) % RISE_WINDOW_SAMPLES;
  return leafMv - oldest;
}

bool rainIsActive(const LeafWetness& w, uint32_t nowMs) {
  return w.everRained && (nowMs - w.lastRainMs) < RAIN_HOLD_MS;
}

WetState chooseState(const LeafWetness& w, uint32_t nowMs) {
  if (!w.wet) return STATE_DRY;
  if (rainIsActive(w, nowMs)) return STATE_RAIN;
  if (w.rainThisWet)   return STATE_WET;
  return STATE_DEW;
}

// Handles one reading (and the gauge tips since the last one).
void addSample(LeafWetness& w, float leafMv, uint32_t tipsNow, uint32_t nowMs) {
  // Rate test: is water arriving?
  w.riseMv     = updateRise(w, leafMv);
  w.rainSignal = (w.riseMv >= RAIN_RISE_MV ? 1 : 0) | (tipsNow > 0 ? 2 : 0);
  if (w.rainSignal) markRain(w, nowMs);

  // Level test: is the leaf wet?
  float aboveMv = leafMv - w.baselineMv;
  if (!w.wet) {
    if (aboveMv < WET_START_MV) w.lastNearBaseMs = nowMs;
    if (aboveMv >= WET_ON_MV) {
      startWetPeriod(w, w.lastNearBaseMs);
      if (w.rainSignal) w.rainThisWet = true;
    } else if (fabs(aboveMv) < BASELINE_FOLLOW_MV && !rainIsActive(w, nowMs)) {
      w.baselineMv += BASELINE_SMOOTHING * aboveMv;       // follow slow drift while dry, never during rain
    }
  } else {
    if (leafMv < w.baselineMv) w.baselineMv = leafMv;     // only matters after a wet boot
    if (aboveMv < WET_OFF_MV) endWetPeriod(w, leafMv, nowMs);
  }

  w.state = chooseState(w, nowMs);
}

// First reading after power-up: use it as the baseline, unless the leaf is
// already clearly wet, in which case fall back to a typical dry level. The cause
// of that wetness is unknown, so it is labelled WET rather than DEW.
void initialiseWetness(LeafWetness& w, float firstMv, uint32_t nowMs) {
  for (int i = 0; i < RISE_WINDOW_SAMPLES; i++) w.history[i] = firstMv;
  w.baselineMv     = firstMv;
  w.lastNearBaseMs = nowMs;

  if (firstMv - DRY_FALLBACK_MV >= WET_ON_MV) {
    w.baselineMv = DRY_FALLBACK_MV;
    startWetPeriod(w, nowMs);
    w.rainThisWet = true;
    w.state       = STATE_WET;
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

// Adds the latest tips to the current hour, moving on to a fresh slot (and
// forgetting the oldest hour) each time an hour passes.
void updateRainHistory(RainHistory& h, uint32_t tipsNow, uint32_t nowMs) {
  const uint32_t HOUR_MS = 3600000UL;
  while (nowMs - h.slotStartMs >= HOUR_MS) {
    h.currentSlot = (h.currentSlot + 1) % RAIN_HISTORY_HOURS;
    h.tipsPerSlot[h.currentSlot] = 0;
    h.slotStartMs += HOUR_MS;
  }
  h.tipsPerSlot[h.currentSlot] += tipsNow;
}

float rainLast24hMm(const RainHistory& h) {
  uint32_t tips = 0;
  for (int i = 0; i < RAIN_HISTORY_HOURS; i++) tips += h.tipsPerSlot[i];
  return tips * RAIN_GAUGE_MM_PER_TIP;
}

// Seconds since the RAIN state ended: 0 while it is raining, -1 if it never has.
long secondsSinceRainStopped(const LeafWetness& w, uint32_t nowMs) {
  if (!w.everRained)            return -1;
  if (rainIsActive(w, nowMs))   return 0;
  return (long)((nowMs - w.lastRainMs - RAIN_HOLD_MS) / 1000);   // RAIN_HOLD_MS after the last signal
}


// =============================================================================
//  Logging (Serial + SD card)
// =============================================================================

// mv = leaf reading now. above_base_mv = mv minus the dry baseline.
// rise10m_mv = change over the last 10 minutes. rain_sig: 0 none, 1 leaf rise,
// 2 gauge tip, 3 both. tips = gauge tips since the previous row; tips_total and
// rain_mm count from power-on.
const char* CSV_HEADER =
  "date,time,uptime_s,mv,state,baseline_mv,above_base_mv,rise10m_mv,rain_sig,"
  "wet_dur_s,since_wet_s,since_rain_s,wet_total_s,temp_c,rh_pct,dewpt_c,"
  "tips,tips_total,rain_mm,since_tip_s";

void formatCsvRow(char* out, size_t outSize, uint32_t nowMs, const UtcTime& utc,
                  float leafMv, const LeafWetness& w, const AirReading& air,
                  const Durations& d, const RainGauge& gauge) {
  char timestamp[24];
  formatTimestamp(utc, timestamp, sizeof timestamp);
  unsigned long sinceTip = gauge.everTipped ? secondsBetween(nowMs, gauge.lastTipMs) : 0;

  snprintf(out, outSize,
           "%s,%lu,%.1f,%s,%.1f,%.1f,%.1f,%u,%lu,%lu,%lu,%lu,%.1f,%.1f,%.1f,%lu,%lu,%.1f,%lu",
           timestamp, (unsigned long)(nowMs / 1000),
           leafMv, stateName(w.state), w.baselineMv, leafMv - w.baselineMv, w.riseMv,
           (unsigned)w.rainSignal,
           (unsigned long)d.wetFor, (unsigned long)d.dryFor,
           (unsigned long)d.sinceRain, (unsigned long)d.totalWet,
           air.temperatureC, air.humidityPercent, air.dewPointC,
           (unsigned long)gauge.tipsNow, (unsigned long)gauge.tipsTotal,
           rainfallMm(gauge), sinceTip);
}

// "raining,rain_24h_mm,seconds_since_rain_stopped", e.g. "0,2.5,7200".
void formatBroadcast(char* out, size_t outSize, const LeafWetness& w,
                     const RainHistory& h, uint32_t nowMs) {
  snprintf(out, outSize, "%d,%.1f,%ld",
           w.state == STATE_RAIN ? 1 : 0,
           rainLast24hMm(h),
           secondsSinceRainStopped(w, nowMs));
}

// Sends the line to every peripheral that is connected and has the UART service.
void broadcastToPeripherals(const char* message) {
  for (int i = 0; i < MAX_PERIPHERALS; i++) {
    if (peripheralUart[i].discovered()) peripheralUart[i].println(message);
  }
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
                 const LeafWetness& w, const Durations& d,
                 const RainGauge& gauge, uint32_t nowMs) {
  char text[28];
  char duration[12];      // longest is "49710d 23h"

  oled.clearBuffer();

  // Row 0: wet/dry state, warnings for missing SD card or GPS time, and
  // P<n> = number of connected Bluetooth peripherals
  snprintf(text, sizeof text, "%s%s%s P%d",
           stateName(w.state),
           sdCardReady ? "" : " !SD",
           utc.valid   ? "" : " !GPS",
           (int)Bluefruit.Central.connected());
  drawRow(0, text);

  // Row 1: leaf reading and dry baseline
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

  // Row 4: time since rain was last detected (leaf rise or gauge tip)
  if (!w.everRained) {
    snprintf(text, sizeof text, "Rain: none seen");
  } else {
    formatDuration(d.sinceRain, duration, sizeof duration);
    snprintf(text, sizeof text, "Rain %s ago", duration);
  }
  drawRow(4, text);

  // Row 5: rain gauge total and time since its last tip
  if (!gauge.everTipped) {
    snprintf(text, sizeof text, "Gauge %.1fmm no tips", rainfallMm(gauge));
  } else {
    formatDuration(secondsBetween(nowMs, gauge.lastTipMs), duration, sizeof duration);
    snprintf(text, sizeof text, "Gauge %.1fmm %s", rainfallMm(gauge), duration);
  }
  drawRow(5, text);

  oled.sendBuffer();
}


// =============================================================================
//  Bluetooth callbacks
// =============================================================================

// Called for every advertising packet seen while scanning. Connects to devices
// that offer the Nordic UART service, until MAX_PERIPHERALS are connected.
void onScanReport(ble_gap_evt_adv_report_t* report) {
  bool hasRoom      = Bluefruit.Central.connected() < MAX_PERIPHERALS;
  bool offersUart   = Bluefruit.Scanner.checkReportForService(report, peripheralUart[0]);
  if (hasRoom && offersUart) Bluefruit.Central.connect(report);
  else                       Bluefruit.Scanner.resume();
}

// Connection handles count from 0, so a handle doubles as the slot number.
void onPeripheralConnect(uint16_t connHandle) {
  bool slotOk = connHandle < MAX_PERIPHERALS;
  if (!slotOk || !peripheralUart[connHandle].discover(connHandle)) {
    Bluefruit.disconnect(connHandle);          // no usable UART service: let go
    return;
  }
  DEBUG_PRINTLN("# BLE peripheral connected");
  if (Bluefruit.Central.connected() < MAX_PERIPHERALS) Bluefruit.Scanner.start(0);   // look for more
}

void onPeripheralDisconnect(uint16_t connHandle, uint8_t reason) {
  (void)connHandle;
  (void)reason;
  DEBUG_PRINTLN("# BLE peripheral disconnected");   // the scanner restarts itself
}


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

void startRainGauge() {
  pinMode(RAIN_GAUGE_PIN, INPUT_PULLUP);              // switch to GND pulls the pin low
  attachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN), onRainGaugeTip, FALLING);
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
  // 0 peripheral connections (we do not advertise), MAX_PERIPHERALS central ones.
  if (!Bluefruit.begin(0, MAX_PERIPHERALS)) {
    DEBUG_PRINTLN("# Bluetooth failed to start");
    return;
  }
  Bluefruit.setTxPower(8);

  for (int i = 0; i < MAX_PERIPHERALS; i++) peripheralUart[i].begin();
  Bluefruit.Central.setConnectCallback(onPeripheralConnect);
  Bluefruit.Central.setDisconnectCallback(onPeripheralDisconnect);

  Bluefruit.Scanner.setRxCallback(onScanReport);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);              // units of 0.625 ms: scan 50 ms of every 100 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                          // scan forever
}

void setup() {
#if DEBUG
  startDebugSerial();
#endif

  startSensorPower();
  startRainGauge();
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
  RainGauge  gauge  = readRainGauge();

  addSample(wetness, leafMv, gauge.tipsNow, nowMs);
  updateRainHistory(rainHistory, gauge.tipsNow, nowMs);
  Durations durations = computeDurations(wetness, nowMs);

  char row[240];
  formatCsvRow(row, sizeof row, nowMs, utc, leafMv, wetness, air, durations, gauge);
  logLine(row);

  char broadcast[32];
  formatBroadcast(broadcast, sizeof broadcast, wetness, rainHistory, nowMs);
  broadcastToPeripherals(broadcast);

  drawDisplay(leafMv, air, utc, wetness, durations, gauge, nowMs);
}
