# Leaf Wetness Logger

A WisBlock (RAK4631) datalogger for the METER **PHYTOS31** leaf wetness sensor, built for vineyard canopy monitoring. Every 10 seconds it measures leaf wetness and air conditions, works out whether the leaf is wet and *why* (rain, dew, or unknown), tracks how long it has been wet, dry, and since the last rain, and logs everything to a microSD card and Bluetooth (and to Serial when `DEBUG` is on), with a live OLED readout.

## Features

- 16-bit leaf wetness measurement through an Adafruit ADS1115, converted to a calibrated 0-100 % wetness value
- Wet/dry detection with hysteresis (no flicker from sensor noise)
- Wet-source guess: **RAIN** (fast rise or drops landing), **DEW** (slow rise, air near dew point), or **WET** (unknown)
- Wet duration, dry duration, time since last rain, and cumulative wet time
- Air temperature, humidity, dew point and dew point depression from a BME680
- Frost flag, because ice on the sensor reads as nearly dry
- UTC timestamps from a u-blox GPS (logging starts before a fix; date/time are blank until one arrives)
- One CSV row per sample, sent to the SD card and Bluetooth UART (and Serial in debug mode)
- 128x64 OLED status display
- Over-the-air firmware updates (BLE DFU)

## Status

| Component | Tested |
|---|---|
| ADS1115 + PHYTOS31 | Yes (wetted repeatedly with a spray bottle) |
| Serial output | Yes |
| GPS timestamps | Yes |
| Bluetooth UART | Yes |
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

### Wiring

```
PHYTOS31   supply  -> 3V3_S
           ground  -> GND
           signal  -> ADS1115 A0

ADS1115    VDD     -> 3V3_S
           GND     -> GND
           SDA/SCL -> WisBlock I2C bus
```

`3V3_S` is the switched sensor rail, enabled by the sketch through `WB_IO2`. It stays on permanently because the ADC and OLED share it. The PHYTOS31 output is a fraction of its supply voltage, so keep the supply steady: calibration is only valid for the supply it was done at.

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

All tunables are at the top of the sketch.

| Constant | Default | Meaning |
|---|---|---|
| `SAMPLE_INTERVAL_MS` | 10000 | Time between samples |
| `RISE_WINDOW_MS` | 60000 | Window used to measure how fast wetness rises |
| `ADC_GAIN` | `GAIN_TWO` | ADS1115 range, +/-2.048 V (see note below) |
| `ADC_AVERAGING_SAMPLES` | 8 | Reads averaged per sample |
| `LEAF_DRY_VOLTS` | 0.350 | Calibration: clean, dry sensor |
| `LEAF_WET_VOLTS` | 1.100 | Calibration: fully wet sensor |
| `WET_ON_PERCENT` | 3.0 | Leaf becomes wet at or above this |
| `WET_OFF_PERCENT` | 1.5 | Leaf becomes dry at or below this |
| `FILM_UPPER_PERCENT` | 20.0 | Upper edge of the `FILM` class |
| `WET_UPPER_PERCENT` | 75.0 | Upper edge of the `WET` class |
| `RAIN_MIN_RISE_PERCENT` | 5.0 | Rise within the window that counts as rain |
| `RAIN_STEP_PERCENT` | 0.5 | A sample-to-sample change this big counts as a landing drop |
| `RAIN_MIN_BIG_STEPS` | 3 | Drops within the window needed to call it rain |
| `RAIN_HOLD_MS` | 300000 | Rain stays "ongoing" this long after the last drop |
| `DEW_MAX_DEPRESSION_C` | 1.5 | Air this close to its dew point makes dew plausible |
| `FROST_BELOW_C` | 1.0 | Air below this raises the frost flag |
| `BLE_DEVICE_NAME` | `PHYTOS31 LOGGER` | Bluetooth advertising name |

### Calibrating

The output range differs between individual sensors, so measure yours:

1. **Dry:** let the sensor sit clean and dry (a dry, still night is ideal). Read the `volts` column and set `LEAF_DRY_VOLTS` slightly above the steady value.
2. **Wet:** spray or submerge the sensing face until it is fully wet and set `LEAF_WET_VOLTS` to the plateau.
3. Re-check `WET_ON_PERCENT` / `WET_OFF_PERCENT`. They should sit a little above the dry noise.

**Why `GAIN_TWO`:** the PHYTOS31 can output up to about half its supply (~1.65 V). `GAIN_FOUR` (+/-1.024 V) would clip a fully wet reading.

## How it works

### Wet / dry

The leaf is **wet** once `wet_pct` reaches `WET_ON_PERCENT` and **dry** again once it falls to `WET_OFF_PERCENT`. The gap between the two is hysteresis, so small fluctuations near the boundary don't toggle the state.

### Source of wetness

When a wet period begins, the sketch guesses its source:

1. Rise of `RAIN_MIN_RISE_PERCENT` or more in the last minute, or at least `RAIN_MIN_BIG_STEPS` sample-to-sample jumps of `RAIN_STEP_PERCENT` or more in the last minute: **RAIN**
2. Otherwise, air temperature within `DEW_MAX_DEPRESSION_C` of the dew point: **DEW**
3. Otherwise: **WET** (unknown, e.g. fog, irrigation, drizzle, or no air sensor)

If rain activity happens later in a wet period (rain on an already dewy leaf), the source is upgraded to **RAIN**.

**Why drop activity, not just rise:** in a real logged rain (WET0002), the leaf climbed only to about 25-40 % and never rose faster than about 8 %/min, so a rise-only rule at 13 % missed every event. What gave rain away was constant jitter: steps of 0.5 % or more every few samples as drops land and run off. A damp or drying sensor changes by 0.2 % or less per sample, so counting big steps separates rain from dew and slow drying.

Every sample with rain activity restarts the "time since rain" clock. Rain counts as **ongoing** until `RAIN_HOLD_MS` after the last drop, then `since_rain_s` counts up from that last drop. Thresholds come from one log, so check them against your own rain data.

This is a heuristic from one leaf sensor plus air conditions. It cannot distinguish every case (see Limitations).

## Output

### OLED (6 rows)

| Row | Shows | Example |
|---|---|---|
| 0 | State, source, warnings | `WET DEW !GPS` |
| 1 | Volts, percent, class | `0.512V 22% WET` |
| 2 | Temp, RH, dew point depression (`dd`), frost flag | `14.2C 95% dd0.8` |
| 3 | Time wet or dry | `Wet for 1h 12m` |
| 4 | Time since rain | `Rain: 4h 10m ago` |
| 5 | Cumulative wet time since boot | `Wet total 6h 30m` |

Warnings on row 0: `!SD` means the SD card is missing or a write failed, `!GPS` means no valid GPS time yet.

### Serial, SD card and Bluetooth

Each sample produces one CSV row. Serial (debug builds only) prints the header at boot, and Bluetooth sends rows only (no header). Every boot starts a **new SD file** (`WET0001.CSV`, `WET0002.CSV`, ...) with the header as its first line; existing files are never appended to. In debug builds the chosen name is printed on Serial at startup (`# Logging to WET0003.CSV`). File names use the 8.3 format so they work with any SD library. Lines starting with `#` on Serial are status messages, not data.

## CSV reference

```
date,time,uptime_s,volts,wet_pct,class,rise_pct_min,wet,source,wet_dur_s,since_wet_s,since_rain_s,wet_total_s,temp_c,rh_pct,dewpt_c,depr_c,frost
```

### Example rows

Dry leaf, GPS time available:

```
2026-10-08,14:03:09,5230,0.3512,0.2,DRY,0.0,0,-,0,3600,0,1200,17.4,58.2,9.3,8.1,0
```

Seventy seconds into a rain event:

```
2026-10-08,14:10:19,5660,0.9150,75.3,DRENCHED,41.2,1,RAIN,70,0,0,1270,15.1,96.4,14.6,0.5,0
```

No GPS fix yet (date and time are empty):

```
,,42,0.3498,-0.2,DRY,0.0,0,-,0,0,0,0,17.4,58.2,9.3,8.1,0
```

### Columns

| # | Column | Unit | Description |
|---|---|---|---|
| 1 | `date` | `YYYY-MM-DD` | UTC date from the GPS. Empty until the GPS has a valid fix. |
| 2 | `time` | `HH:MM:SS` | UTC time from the GPS. Empty until the GPS has a valid fix. |
| 3 | `uptime_s` | s | Seconds since power-on. Always present. Wraps after about 49.7 days. |
| 4 | `volts` | V | PHYTOS31 output at ADS1115 A0, average of 8 reads, 4 decimals. The raw measurement. |
| 5 | `wet_pct` | % | `(volts - LEAF_DRY_VOLTS) / (LEAF_WET_VOLTS - LEAF_DRY_VOLTS) * 100`. Not clamped: noise can give slightly negative values and a wetter-than-calibration sensor can exceed 100. |
| 6 | `class` | text | Coarse level of `wet_pct`: `DRY` below 1.5, `FILM` below 20, `WET` below 75, `DRENCHED` at or above 75. |
| 7 | `rise_pct_min` | %-points | `wet_pct` now minus `wet_pct` 60 s ago. Positive means wetting, negative means drying. Zero for the first minute after boot. |
| 8 | `wet` | 0 / 1 | Wet state with hysteresis (on at 3 %, off at 1.5 %). |
| 9 | `source` | text | Guessed cause of the current wet period: `RAIN`, `DEW`, `WET` (unknown), or `-` when dry. |
| 10 | `wet_dur_s` | s | Length of the current wet period. 0 when dry. |
| 11 | `since_wet_s` | s | Time since the last wet period ended. 0 while wet or if the leaf has never been wet since boot. |
| 12 | `since_rain_s` | s | Time since the last rain activity (last drop or fast rise). Counts up from 0 during rain, so it stays under 300 while rain is ongoing; 0 also means none has been seen since boot. |
| 13 | `wet_total_s` | s | Total wet time since boot, including the current wet period. |
| 14 | `temp_c` | degC | Air temperature from the BME680. `nan` if the sensor is missing. |
| 15 | `rh_pct` | %RH | Relative humidity. `nan` if the sensor is missing. |
| 16 | `dewpt_c` | degC | Dew point (Magnus formula). `nan` if the sensor is missing. |
| 17 | `depr_c` | degC | Dew point depression, `temp_c - dewpt_c`. Small values mean the air is close to condensing. `nan` if the sensor is missing. |
| 18 | `frost` | 0 / 1 | 1 when `temp_c` is below 1 degC. Ice reads as nearly dry, so treat a "dry" reading as unreliable when this is 1. |

### Reading the columns together

- **`class` vs `wet`:** `wet` has hysteresis and `class` does not, so a reading between 1.5 and 3 % can be class `FILM` while `wet` is still 0. `wet` is the one to use for durations.
- **Zeros are ambiguous.** `since_wet_s = 0` means "currently wet" or "never wet"; `since_rain_s = 0` means "rain just now" or "no rain seen". Check `source` to tell which.
- **Dew vs rain at night:** `source=DEW` with a small `depr_c` and a gentle `rise_pct_min` is dew. A fast `rise_pct_min`, or jittery `wet_pct` readings, flip it to `RAIN` regardless of `depr_c`.
- **Frozen sensor:** `frost=1` with a low `wet_pct` does not mean a dry leaf.
- **Disease models** need wet *hours* and temperature, not instantaneous wetness: use `wet_dur_s` (or sum the `wet` column) with `temp_c`.

### Quick analysis (Python)

```python
import pandas as pd

df = pd.read_csv("WET0001.CSV", parse_dates={"timestamp": ["date", "time"]})
df = df.dropna(subset=["timestamp"]).set_index("timestamp")

interval_h = 10 / 3600                      # SAMPLE_INTERVAL_MS in hours
daily_wet_hours = df["wet"].resample("D").sum() * interval_h
print(daily_wet_hours)
```

## Limitations

- **No real-time clock.** Durations and `uptime_s` restart at every reset. Rows only carry real dates once the GPS has a fix.
- **One sensor cannot always separate dew, fog, drizzle and irrigation.** The `source` column is a best guess.
- **A sensor is not a leaf.** Thermal mass, angle and exposure differ, so onset and drying times will not match your canopy exactly. Mount it at leaf angle in the canopy.
- **Residue and contamination** (spray residue, dust, bird droppings) can shift the dry baseline. Re-check the dry voltage after spraying.
- **Ice reads as dry.** See the `frost` column.
- **CSV schema changes between versions.** Files from earlier sketch versions (`wetness.csv`) use a different column layout. New files are always written with the current header.

## Troubleshooting

| Symptom | Check |
|---|---|
| OLED shows `# ADS1115 not found` (and Serial repeats it when `DEBUG` is 1) | ADC wiring, power on 3V3_S, I2C address `0x48`. The sketch stops until the ADC is found. |
| `# RAK1906 not found` | Module seated in its slot. Logging continues with `nan` air values. |
| `# GPS not found` | Module and I2C wiring. Logging continues with blank date/time. |
| `# SD card not found` / `!SD` on OLED | Card seated and FAT-formatted. Logging continues on Bluetooth (and Serial in debug builds). |
| `# No free log file name on SD card` | All 9999 file names are used. Archive or delete old `WET*.CSV` files. |
| Blank or upside-down OLED | `U8G2_R2` rotates 180 degrees (use `U8G2_R0` for the other way). Confirm the I2C address and module. |
| `wet_pct` always high or negative | Recalibrate `LEAF_DRY_VOLTS` / `LEAF_WET_VOLTS` |
| `!GPS` never clears | GPS needs a clear sky view. Allow several minutes for a first fix. |

## Roadmap

- Real-time clock fallback so durations survive resets
- Test and document the OLED and SD paths
- Optional tipping-bucket rain gauge for a true "time since rain"

## License

Add a license of your choice (MIT is a common default for hardware/Arduino projects).
