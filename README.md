# Leaf Wetness Logger

A WisBlock (RAK4631) datalogger for the METER **PHYTOS31** leaf wetness sensor, built for vineyard canopy monitoring. It reads the sensor every 10 seconds, works out whether the leaf is wet and *why* (rain, dew, or unknown) from how far and how fast the signal has risen, tracks how long it has been wet, dry, and since the last rain, and logs everything to a microSD card (and to Serial when `DEBUG` is on) and sends a short rain summary to up to four Bluetooth peripherals, with a live OLED readout. A tipping bucket rain gauge is logged alongside, as ground truth for rain events.

The wetness logic (v8, extended in v10) is deliberately simple: a **level test** against a self-learned dry baseline says whether the leaf is wet, and a **rate test** over the last 10 minutes (plus how much the reading is jumping about, and the rain gauge) says whether water is arriving. It replaces the Binks-style 15-minute-step method of v6/v7, which hid the light-rain events in `WET0003.CSV`. See [How it works](#how-it-works).

## Features

- 16-bit leaf wetness measurement through an Adafruit ADS1115, in millivolts of sensor output
- Wet/dry from the sensor's height above a dry baseline it learns itself (no calibration endpoints), and rain from a rise of a few mV in 10 minutes or a gauge tip: **RAIN**, **WET** (rain stopped, leaf still wet), **DEW** (wet with no rain seen), **DRY**
- Wet duration, dry duration, time since last rain, and cumulative wet time
- Air temperature, humidity and dew point from a BME680 (logged for context, not used to classify)
- Tipping bucket rain gauge (TR-525M) on `WB_IO3`: interrupt-driven tip counting with debounce, logged as tips per sample, running total, rainfall in mm and time since the last tip. A tip also counts as a rain signal for the state logic
- UTC timestamps from a u-blox GPS on every row: startup blocks until the GPS has the date and time, and the OLED shows `Waiting for GPS fix` with the satellite count and time waited
- One CSV row per sample, sent to the SD card (and Serial in debug mode)
- Bluetooth central: connects to up to 4 Nordic UART peripherals and sends each a rain summary line after every sample (see [Bluetooth broadcast](#bluetooth-broadcast))
- A new SD file on every boot, so earlier logs are never touched
- 128x64 OLED status display

## Status

| Component | Tested |
|---|---|
| ADS1115 + PHYTOS31 | Yes (spray bottle, and a real rain log, `WET0002.CSV`) |
| Serial output | Yes |
| GPS timestamps | Yes |
| Bluetooth central + broadcast (v9) | Not yet. Compiled and the message values checked in a desktop test build only |
| Wetness logic (v8) | Run live on the device: `WET0003.CSV` and `WET0011.CSV` (65 gauge tips, 6.5 mm) |
| Activity rain signal and GPS wait (v10) | Replayed offline against three logs; GPS wait checked in a desktop test build only, not yet run live |
| TR-525M rain gauge | Logged live in `WET0003.CSV` (one tip). Tip counting and debounce are also checked in a desktop test build |
| RAK1921 OLED | Not yet |
| microSD logging | Not yet |

## Hardware

| Part | Purpose | Notes |
|---|---|---|
| RAK4631 WisBlock Core + base board | MCU, BLE | nRF52840 |
| METER PHYTOS31 | Leaf wetness | Analog output, ratiometric to its supply |
| Adafruit ADS1115 | 16-bit ADC | I2C address `0x48` |
| RAK1906 | Temperature / humidity (BME680) | I2C address `0x76` |
| RAK1921 | 128x64 OLED (SSD1306) | I2C |
| RAK15002 | microSD | |
| u-blox GNSS module | UTC time | I2C, e.g. RAK12500 |
| Texas Electronics TR-525M | Tipping bucket rain gauge | Reed switch, 0.1 mm per tip, on `WB_IO3` |

### Wiring

```
PHYTOS31   supply  -> 3V3_S
           ground  -> GND
           signal  -> ADS1115 A0

ADS1115    VDD     -> 3V3_S
           GND     -> GND
           SDA/SCL -> WisBlock I2C bus

TR-525M    reed switch lead 1 -> WB_IO3
           reed switch lead 2 -> GND
```

The reed switch has no polarity. The sketch turns on the pin's internal pull-up, so no external resistor is needed. Check which two of the gauge's wires are the switch against its manual.

`3V3_S` is the switched sensor rail, enabled by the sketch through `WB_IO2`. It stays on permanently because the ADC and OLED share it. The PHYTOS31 output is a fraction of its supply voltage, so keep the supply steady: the millivolt thresholds below are only valid for the supply they were chosen at.

## Software setup

1. Install the **RAK nRF52 BSP** in the Arduino IDE and select **WisCore RAK4631 Board**.
2. Install these libraries from the Library Manager:
   - Adafruit ADS1X15
   - U8g2
   - Adafruit BME680 Library (and Adafruit Unified Sensor)
   - SparkFun u-blox GNSS Arduino Library

   `SD.h`, `Wire.h` and `bluefruit.h` come with the BSP.
3. Put the sketch in a folder with the same name as the `.ino` file (an Arduino requirement), open it, and upload.
4. With `#define DEBUG 1` (the default), open the Serial monitor at **115200 baud**. The sketch waits up to 3 seconds for it to connect. For deployment set `#define DEBUG 0` near the top of the sketch: Serial is then never started or used, and none of the status messages exist in the build.

## Configuration

All tunables are at the top of the sketch. Millivolt values are the sensor's output at the ADS1115 input.

| Constant | Default | Meaning |
|---|---|---|
| `SAMPLE_INTERVAL_MS` | 10000 | Time between readings and CSV rows |
| `ADC_GAIN` | `GAIN_TWO` | ADS1115 range, +/-2.048 V (see note below) |
| `ADC_AVERAGING_SAMPLES` | 8 | Reads averaged per sample |
| `WET_ON_MV` | 20 | Wet once the reading is this far above the dry baseline |
| `WET_OFF_MV` | 15 | Dry again below this (hysteresis) |
| `WET_START_MV` | 3 | The wet period is back-dated to when the signal was last within this of the baseline |
| `BASELINE_FOLLOW_MV` | 8 | While dry, drift smaller than this is followed (never during rain) |
| `BASELINE_SMOOTHING` | 0.05 | How fast the baseline follows, per sample |
| `DRY_FALLBACK_MV` | 350 | Baseline if the logger powers up on an already wet leaf |
| `RISE_WINDOW_SAMPLES` | 60 | Window for the rate test (60 x 10 s = 10 min) |
| `RAIN_RISE_MV` | 4 | A rise this big within the window counts as rain |
| `ZIGZAG_SAMPLES` | 6 | Samples in the zig-zag window (6 x 10 s = 1 minute). Short on purpose, so the signal stops soon after the rain does |
| `RAIN_ACTIVITY_MV` | 0.3 | Zig-zag per sample over that minute that counts as rain (see `activity_mv`) |
| `RAIN_HOLD_MS` | 30 min | Rain stays "active" this long after the last signal |
| `RAIN_GAUGE_PIN` | `WB_IO3` | Pin the rain gauge reed switch is wired to |
| `RAIN_GAUGE_MM_PER_TIP` | 0.1 | Rainfall per tip. 0.1 mm for the TR-525M |
| `TIP_DEBOUNCE_MS` | 50 | Closures this soon after a counted tip are ignored |
| `MAX_PERIPHERALS` | 4 | Bluetooth peripherals the logger stays connected to |
| `RAIN_HISTORY_HOURS` | 24 | Window for the rain total in the broadcast |

**Why `GAIN_TWO`:** the PHYTOS31 can output up to about half its supply (~1.65 V). `GAIN_FOUR` (+/-1.024 V) would clip a fully wet reading.

**Check the thresholds against your own data.** `RAIN_RISE_MV`, `RAIN_ACTIVITY_MV` and the wet thresholds were chosen from three logs. Only `WET0011.CSV` has a gauge record that covers the rain it contains, so treat them as a good starting point, not a calibration. The `rain_sig`, `rise10m_mv` and `activity_mv` columns exist so you can re-tune them offline.

## How it works

Two independent questions are answered at every 10-second sample.

### Is the leaf wet? (level test)

The **baseline** is the dry level. While the leaf is dry it creeps along with slow drift (temperature, dirt) but is frozen whenever rain is active. The leaf is **wet** once the reading is `WET_ON_MV` above the baseline, and **dry** again below `WET_OFF_MV`. The wet period is back-dated to the last moment the signal was within `WET_START_MV` of the baseline, so `wet_dur_s` includes the slow start. When a wet period ends, the baseline is re-learned from where the signal settled.

### Is water arriving? (rate test)

A sample is a **rain signal** (`rain_sig` column) if any of these is true:

- the reading has risen by `RAIN_RISE_MV` or more over the last 10 minutes;
- the reading has been zig-zagging: over the last minute, the distance it travelled minus its net change, per sample (`activity_mv`), is at least `RAIN_ACTIVITY_MV`. This catches steady or heavy rain on an already wet sensor, where the level has stopped rising but drops keep landing and running off. A smooth rise or fall, such as a leaf drying quickly indoors, scores zero;
- the gauge tipped.

Dew, temperature drift and drying are far slower and smoother than that. Each signal restarts `since_rain_s`, and rain is "active" for `RAIN_HOLD_MS` after the last one.

### States

| State | Meaning |
|---|---|
| `DRY` | Leaf at its baseline |
| `RAIN` | Wet, and a rain signal occurred in the last `RAIN_HOLD_MS` |
| `WET` | Wet, rain was seen in this wet period but not recently (also used when the logger boots on a wet leaf) |
| `DEW` | Wet, and no rain signal at all during this wet period |

### Start-up

At power-up the first reading is the baseline. If it is already `WET_ON_MV` or more above `DRY_FALLBACK_MV`, the leaf is treated as wet with `DRY_FALLBACK_MV` as the baseline.

### What the logs showed

`WET0003.CSV` (v7 firmware, about 15 h, 0.1 mm gauge):

- The sensor was installed and booted outdoors in wet conditions. It read a dry 361-364 mV for the first 40 minutes, then rose in a small step at 06:12 UTC and a fast ramp (+57 mV in 5 minutes) from 06:25, flattening near 505 mV by 07:05. This was observed light rain. The gauge recorded nothing for it, because the total was under one 0.1 mm tip.
- The leaf stayed wet all night and through a cloudy morning. It began drying only when the sun came out near the end of the log (about 17:00-20:50 UTC, 505 to 439 mV) and was still not dry when the log was pulled. So the sustained high reading was real wetness, not a stuck sensor.
- The only gauge tip was at 09:37 UTC (01:37 local), and the leaf had risen about 4 mV a few minutes earlier. A light event at about 09:30-10:35 raised it a further 20 mV in small 3-5 mV steps.
- Once wet, the signal moves in discrete steps of a few mV (up when water arrives, down by 10-14 mV when a drop runs off, for example at 13:15-13:25). It is quiet otherwise: the 60-second noise is under 0.2 mV.
- With the sensor wet, mV correlated with air temperature at -0.83, and the evening decline (505 to 439 mV from 17:00 to 20:50) happens while the air warms. Drying is slow.
- v7's 15-minute steps averaged the first ramp into one +15.7 mV step, which is below its 100 mV rain threshold, so it was labelled WET, not RAIN, and never ended.

Replaying v8 on this log: DRY to RAIN at 06:26 UTC (back-dated to 06:12), then candidate rain signals at 08:06, 09:31, 11:33, 12:30 and 14:45. The 08:06, 11:33, 12:30 and 14:45 signals are single steps of 3-5 mV and **have no independent confirmation**, so they may be light rain or may be droplet movement or temperature. On `WET0002.CSV` (heavy rain) v8 detects wetting at 22:03 UTC, with the first rain signal at about 21:41.

`WET0011.CSV` (v8 firmware, about 10 h, 65 tips = 6.5 mm) is the first log with a gauge record that covers real rain:

- **Event 1, from about 06:36 UTC.** The leaf rose from 364 to 560 mV in about an hour as the first 1.4 mm fell, then stayed at 560 mV through the lull. The first tip (06:55) came about 20 minutes after the leaf began to climb.
- **Lull, about 08:30-09:50.** The gauge was silent and the leaf flat at 560 mV, with 1.2 hours and no false rain signals. Wet, rain-free leaf noise is under 0.1 mV.
- **Event 2, from about 09:55.** The leaf crept up 16 mV about 40 minutes before the next tip (10:34), which looks like light rain that was too little for the gauge. Heavy rain followed from about 11:30 to 15:30 (51 tips, tips every 2-5 minutes).
- **The reading is not capped at 560 mV.** It rose to 650-680 mV in the heaviest rain, so more water on the sensor means a higher reading.
- **Every tip fell inside a RAIN period**, and the state ended RAIN 30 minutes after the last signal. The longest gap between rain signals inside an event was 27 minutes (the end of event 1), which is why `RAIN_HOLD_MS` is 30 minutes.
- **Leaf activity tracks rain rate.** Per 10 minutes, the average change between samples (the first version of `activity_mv`) was 0.4 mV with no tips, 0.9 with one tip, about 2 with two or three tips, and 3.4 with four or five (correlation 0.75). The zig-zag version used in v10 correlates just as well (0.76, measured over 10-minute windows) but ignores smooth trends, which is why v10 uses it as a rain signal, and it may later give a rain-rate estimate finer than the gauge's 0.1 mm.
- **No other signal was needed for detection.** Replaying v10 on this log gives the same RAIN/WET periods as v8: DRY to RAIN at 06:46, RAIN to WET at 08:57, WET to RAIN at 09:55. The activity signal fills gaps (steady rain, and a falling reading during run-off) and changes the older logs: it extends WET0003's first RAIN period to 07:53, and finds extra, unconfirmed RAIN periods there at 12:25 and 15:30, and no longer treats the smooth 33 mV fall at 01:33 UTC in `WET0002.CSV` as rain.

### Rain gauge

Each tip of the bucket closes a reed switch, which pulls `WB_IO3` low. A falling-edge interrupt counts it, ignoring any closure within `TIP_DEBOUNCE_MS` of the last counted tip. According to the LI-COR and Campbell Scientific documentation for this gauge, the switch stays closed for about 135 ms per tip and bounces for under 1 ms, so counting only the falling edge and a 50 ms window gives one count per tip. Every 10-second row logs the tips since the previous row, the running total (and rainfall in mm), and the seconds since the last tip.

A tip counts as a rain signal (`rain_sig` bit 2) and restarts `since_rain_s`. A tip only happens at 0.1 mm, so drizzle and the start of light rain are invisible to the gauge: the leaf sensor responds first. Counts restart at zero on every reset.

### Dew vs the air sensor

The BME680 is inside a sealed enclosure, so its temperature and humidity do not match the air at the leaves, and **no classification uses it**. It is logged for context only. Once a shielded sensor (e.g. SHT31) is mounted at canopy height, a humidity or dew-point check can be added.

This is a heuristic from one leaf sensor. It cannot distinguish every case (see Limitations).

## Output

### OLED (6 rows)

| Row | Shows | Example |
|---|---|---|
| 0 | State, warnings, connected BLE peripherals | `RAIN !GPS P2` |
| 1 | Leaf reading and baseline, in mV | `542 mV base 354` |
| 2 | Temp, RH, dew point | `14.2C 95% dp13.5` |
| 3 | Time wet or dry | `Wet for 1h 12m` |
| 4 | Time since rain was last detected (leaf rise or gauge tip) | `Rain 4h 10m ago` |
| 5 | Rain gauge total and time since its last tip | `Gauge 12.3mm 4m 05s` |

Warnings on row 0: `!SD` means the SD card is missing or a write failed, `!GPS` means the GPS has lost its time since startup.

While waiting for the GPS at startup the display shows `Waiting for GPS fix`, the number of satellites in view, and how long it has been waiting. Nothing is logged until the fix arrives.

### Serial and SD card

Each sample produces one CSV row. Serial (debug builds only) prints the header at boot. Every boot starts a **new SD file** (`WET0001.CSV`, `WET0002.CSV`, ...) with the header as its first line; existing files are never appended to. In debug builds the chosen name is printed on Serial at startup (`# Logging to WET0003.CSV`). File names use the 8.3 format so they work with any SD library. Lines starting with `#` on Serial are status messages, not data.

### Bluetooth broadcast

The logger is a BLE **central**: it scans for devices that offer the Nordic UART service, connects to up to `MAX_PERIPHERALS` (4) of them, and after every sample writes each one a single line (10 s apart by default):

```
raining,rain_24h_mm,seconds_since_last_rain
0,2.5,7200
```

| Field | Meaning |
|---|---|
| `raining` | `1` while the state is `RAIN`, otherwise `0` |
| `rain_24h_mm` | Rain gauge total over the last 24 hours, in 0.1 mm steps. Counts from power-on if the logger has run for less than 24 hours. The window is in whole hours, so it covers 23-24 hours. |
| `seconds_since_last_rain` | Seconds since the last rain signal (leaf or gauge), the same value as `since_rain_s` in the CSV and the OLED. It keeps counting while `raining` is still 1, because rain stays "active" for `RAIN_HOLD_MS` after the last signal. `-1` if no rain has been seen since power-on. |

The example means: not raining, 2.5 mm in the past 24 hours, last rain signal 2 hours ago. The rain total comes from the gauge only, which cannot see drizzle under 0.1 mm, so light leaf-sensor rain events will show `raining = 1` without adding to the total.

Because the logger is a central it no longer advertises, so it cannot be reached by a phone, and **BLE firmware updates (DFU) and the old UART row stream are gone**. Upload over USB.

## CSV reference

```
date,time,uptime_s,mv,state,baseline_mv,above_base_mv,rise10m_mv,activity_mv,rain_sig,wet_dur_s,since_wet_s,since_rain_s,wet_total_s,temp_c,rh_pct,dewpt_c,tips,tips_total,rain_mm,since_tip_s
```

Rows come every 10 seconds and every column is decided at that rate. `tips` is rain gauge tips since the previous row.

### Example rows

Dry leaf, GPS time available:

```
2026-11-18,05:55:50,370,363.6,DRY,363.6,0.0,0.0,0.01,0,0,0,0,0,10.6,66.2,4.6,0,0,0.0,0
```

The first gauge tip of the rain in `WET0011.CSV` (leaf and gauge both signal rain):

```
2026-11-18,06:55:49,3850,453.4,RAIN,364.3,89.1,76.9,1.47,3,750,0,0,750,10.0,61.2,2.8,1,1,0.1,1
```

### Columns

| # | Column | Unit | Description |
|---|---|---|---|
| 1 | `date` | `YYYY-MM-DD` | UTC date from the GPS. Empty only if the GPS loses its time after startup. |
| 2 | `time` | `HH:MM:SS` | UTC time from the GPS. Empty only if the GPS loses its time after startup. |
| 3 | `uptime_s` | s | Seconds since power-on. Always present. Wraps after about 49.7 days. |
| 4 | `mv` | mV | PHYTOS31 output at ADS1115 A0, average of 8 reads. The live measurement. |
| 5 | `state` | text | `DRY`, `DEW`, `RAIN` or `WET`. Updated every sample. |
| 6 | `baseline_mv` | mV | The dry level. Follows slow drift while dry; frozen while wet or while rain is active. |
| 7 | `above_base_mv` | mV | `mv` minus `baseline_mv`. The level the wet test uses. |
| 8 | `rise10m_mv` | mV | `mv` minus the reading 10 minutes earlier. The rate the rain test uses. |
| 9 | `activity_mv` | mV | Zig-zag over the last minute: the distance the reading travelled minus its net change, divided by the 5 steps. 0 for a smooth rise or fall. The test uses this for rain. |
| 10 | `rain_sig` | 0-3 | Rain signal this sample: 1 = leaf (rise reached `RAIN_RISE_MV` or activity reached `RAIN_ACTIVITY_MV`), 2 = gauge tip, 3 = both, 0 = none. |
| 11 | `wet_dur_s` | s | Length of the current wet period, back-dated to when the signal began leaving the baseline. 0 when dry. |
| 12 | `since_wet_s` | s | Time since the last wet period ended. 0 while wet or if the leaf has never been wet since boot. |
| 13 | `since_rain_s` | s | Time since the last rain signal. 0 means none has been seen since boot. |
| 14 | `wet_total_s` | s | Total wet time since boot, including the current wet period. |
| 15 | `temp_c` | degC | Air temperature from the BME680 (inside the enclosure). `nan` if the sensor is missing. |
| 16 | `rh_pct` | %RH | Relative humidity from the BME680. `nan` if the sensor is missing. |
| 17 | `dewpt_c` | degC | Dew point (Magnus formula) from the two values above. `nan` if the sensor is missing. |
| 18 | `tips` | count | Rain gauge tips since the previous row (normally 10 s earlier). |
| 19 | `tips_total` | count | Tips since power-on. |
| 20 | `rain_mm` | mm | `tips_total` x 0.1 mm. Rainfall since power-on. |
| 21 | `since_tip_s` | s | Seconds since the last counted tip. 0 if there has been none since boot (check `tips_total`). |

### Reading the columns together

- **Zeros are ambiguous.** `since_wet_s = 0` means "currently wet" or "never wet"; `since_rain_s = 0` means "no rain seen" (it counts up from 0 right after rain, so check `state`).
- **`since_rain_s` means "last rain signal", not "last drop".** A rain signal needs a rise of `RAIN_RISE_MV` in 10 minutes, so steady rain on an already saturated sensor may not register.
- **Wetness can last for hours after rain.** `state` goes from `RAIN` to `WET` 30 minutes after the last signal, and stays wet until the reading returns near `baseline_mv`. Use `since_rain_s` for how long ago it rained.
- **Tuning offline:** filter `rain_sig > 0` and compare against `tips` and your own weather record. If dry-weather noise or temperature drift produces false signals, raise `RAIN_RISE_MV`; if known light rain is missed, lower it.
- **Temperature:** in `WET0003.CSV` the dry reading drifted by roughly -1 mV per degC (rough, from a few degrees of range). Keep `WET_OFF_MV` above that drift over a day's temperature swing.
- **Disease models** need wet *hours* and temperature, not instantaneous wetness: use `wet_dur_s` (or sum rows where `state` is not `DRY`) with an air temperature from a representative sensor.

### Quick analysis (Python)

```python
import pandas as pd

df = pd.read_csv("WET0001.CSV", parse_dates={"timestamp": ["date", "time"]})
df = df.dropna(subset=["timestamp"]).set_index("timestamp")

interval_h = 10 / 3600                      # SAMPLE_INTERVAL_MS in hours
wet = df["state"] != "DRY"
daily_wet_hours = wet.resample("D").sum() * interval_h
print(daily_wet_hours)
```

## Limitations

- **Rain gauge counts are since boot.** A reset sets `tips_total` and `rain_mm` back to zero. A bucket that has not yet tipped holds some water, so the log lags the actual rain by up to 0.1 mm and by the time the next tip takes.
- **No real-time clock.** Durations and `uptime_s` restart at every reset, and so do the baseline and totals. Real dates come from the GPS, and startup waits for it.
- **No GPS, no logging.** Startup blocks until the GPS has the date and time, so a missing or blocked antenna stops the logger after boot.
- **One sensor cannot always separate dew, fog, drizzle and irrigation.** The `state` column is a best guess from the rate of rise.
- **Wet surfaces dry slowly.** In `WET0003.CSV` the sensor stayed 80-150 mV above its dry level for 14 hours of dark, cloudy, humid weather, and only started to dry in sun.
- **The gauge cannot see light rain.** A tip needs 0.1 mm, and a dry funnel absorbs some water first, so drizzle may never tip. Light events in `WET0003.CSV` were visible only on the leaf sensor.
- **Rain thresholds are estimates.** Only one tip exists to confirm a rain signal, so most of the small-step events are unconfirmed.
- **A sensor is not a leaf.** Thermal mass, angle and exposure differ, and drops run off a tilted sensor, so onset and drying times will not match your canopy exactly. Mount it at leaf angle in the canopy.
- **Residue and contamination** (spray residue, dust, bird droppings) change the signal. The floating baseline absorbs slow drift, but clean the sensor periodically (Binks et al. cleaned theirs about every three months).
- **Thresholds are not tuned to your site.** They come from the paper and one log; compare against your own rain events.
- **CSV schema changes between versions.** Files from earlier sketch versions use different columns. New files are always written with the current header.

## Troubleshooting

| Symptom | Check |
|---|---|
| OLED shows `# ADS1115 not found` (and Serial repeats it when `DEBUG` is 1) | ADC wiring, power on 3V3_S, I2C address `0x48`. The sketch stops until the ADC is found. |
| `# RAK1906 not found` | Module seated in its slot. Logging continues with `nan` air values. |
| `# GPS not found` | Module and I2C wiring. The logger stops with this message, because every row needs a timestamp. |
| `# SD card not found` / `!SD` on OLED | Card seated and FAT-formatted. Logging continues on Serial in debug builds. |
| `# No free log file name on SD card` | All 9999 file names are used. Archive or delete old `WET*.CSV` files. |
| Blank or upside-down OLED | `U8G2_R2` rotates 180 degrees (use `U8G2_R0` for the other way). Confirm the I2C address and module. |
| State stuck on `DRY` during rain | Check `above_base_mv` and `rise10m_mv`. Wetting needs `WET_ON_MV` above baseline; light rain that never gets there stays `DRY` but still logs `rain_sig`. |
| State stuck on a wet state for hours | Normal after rain: it ends only when `above_base_mv` falls below `WET_OFF_MV`. If the sensor has not truly dried, the baseline may also be stale: restart to re-learn it on a dry sensor. |
| `tips_total` stays 0 in rain | Switch wiring (WB_IO3 and GND), and that the funnel is level and unblocked. Short the two leads together and confirm a tip is counted. |
| Peripheral never connects | It must advertise the Nordic UART service. `P<n>` on OLED row 0 is the number connected (max 4). |
| Last field of the Bluetooth line stays 0 after rain | It reads 0 for `RAIN_HOLD_MS` (30 minutes) after the last rain signal. For bench tests set `RAIN_HOLD_MS` to 2 minutes. The leaf signal itself clears within about a minute of the last drop landing (`ZIGZAG_SAMPLES`), but the 10-minute rise test can hold on for up to 10 minutes after a rise. |
| Receiver shows 4294967295 for "no rain yet" | The logger sends `-1`. Store that field in a signed variable (`long` or `int32_t`) on the receiver, and compare with the `# BLE tx:` line the logger prints to Serial (`DEBUG 1`). |
| Stuck on `Waiting for GPS fix` | The GPS needs a clear sky view. Check the satellite count on the display, and allow several minutes for a first fix. |

## Roadmap

- Shielded SHT31 at canopy height, then a humidity/dew-point check for DEW vs fog
- Real-time clock fallback so durations survive resets
- Test and document the OLED and SD paths
- Compare `rain_sig` against a nearby weather station to confirm or retune `RAIN_RISE_MV`
- Add a temperature correction to the baseline once the shielded SHT31 is fitted

## References

- Binks, O., Carle, H., Coughlin, I., da Costa, A. L., & Meir, P. (2021). [Measuring the vertical profile of leaf wetness in a forest canopy](https://pmc.ncbi.nlm.nih.gov/articles/PMC8374337). *MethodsX* 8:101332. (v6/v7 followed this paper; v8 onwards does not.)
- Kleb, M., Merkt, N., & Zorb, C. (2022). [New Aspects of In Situ Measurements for Downy Mildew Forecasting](https://www.mdpi.com/2223-7747/11/14/1807/htm). *Plants* 11(14):1807.
- Sentelhas, P. C., et al. (2005). [Spatial variability of leaf wetness duration in different crop canopies](https://ippc2.orst.edu/len/articles_canopy_conversion/sentelhas_etal_2005_full.pdf). *Int. J. Biometeorology* 49:363-370.

## License

MIT License
