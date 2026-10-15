# Leaf Wetness Logger

A WisBlock (RAK4631) datalogger for the METER **PHYTOS31** leaf wetness sensor, built for vineyard canopy monitoring. It reads the sensor every 10 seconds, works out whether the leaf is wet and *why* (rain, dew, or unknown) from how fast the signal rises, tracks how long it has been wet, dry, and since the last rain, and logs everything to a microSD card and Bluetooth (and to Serial when `DEBUG` is on), with a live OLED readout.

The wetness algorithm follows Binks et al. (2021), who used the same sensor: no fixed wet/dry thresholds, a floating per-event baseline, and rain/dew told apart by rate of rise. See [How it works](#how-it-works).

## Features

- 16-bit leaf wetness measurement through an Adafruit ADS1115, in millivolts of sensor output
- Binks-style detection on 15-minute averages: **RAIN**, **DEW**, or **WET** (wet but not shaped like either)
- No calibration endpoints: the baseline is the reading just before each wetting, so drift and dirt do not matter
- Wet duration, dry duration, time since last rain, and cumulative wet time
- Air temperature, humidity and dew point from a BME680 (logged for context, not used to classify)
- UTC timestamps from a u-blox GPS (logging starts before a fix; date/time are blank until one arrives)
- One CSV row per sample, sent to the SD card and Bluetooth UART (and Serial in debug mode)
- A new SD file on every boot, so earlier logs are never touched
- 128x64 OLED status display
- Over-the-air firmware updates (BLE DFU)

## Status

| Component | Tested |
|---|---|
| ADS1115 + PHYTOS31 | Yes (spray bottle, and a real rain log, `WET0002.CSV`) |
| Serial output | Yes |
| GPS timestamps | Yes |
| Bluetooth UART | Yes |
| Wetness algorithm (v6) | Replayed offline against `WET0002.CSV`; not yet run live on the device |
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
| `STEP_INTERVAL_MS` | 900000 | Length of the averaging step the algorithm works on (15 min) |
| `ADC_GAIN` | `GAIN_TWO` | ADS1115 range, +/-2.048 V (see note below) |
| `ADC_AVERAGING_SAMPLES` | 8 | Reads averaged per sample |
| `RAIN_RISE_MV` | 100 | Rise in one step that means rain |
| `DEW_MIN_RISE_MV` | 1 | Dew: each step must rise by at least this... |
| `DEW_MAX_RISE_MV` | 15 | ...and by no more than this... |
| `DEW_MIN_STEPS` | 5 | ...for this many steps in a row (75 min) |
| `WET_END_FACTOR` | 1.10 | The leaf is dry again below this multiple of the baseline |
| `QUIET_CHANGE_MV` | 0.5 | A step changing by less than this counts as levelled off |
| `DEW_QUIET_STEPS` | 5 | Dew must also be levelled off this many steps in a row to end |
| `RAIN_RESURGE_MV` | 20 | During rain, a rise this big restarts "time since rain" (not in the paper) |
| `DRY_FALLBACK_MV` | 350 | Baseline if the logger powers up on an already wet leaf (not in the paper) |
| `BLE_DEVICE_NAME` | `PHYTOS31 LOGGER` | Bluetooth advertising name |

**Why `GAIN_TWO`:** the PHYTOS31 can output up to about half its supply (~1.65 V). `GAIN_FOUR` (+/-1.024 V) would clip a fully wet reading.

**Check the thresholds against your own data.** The 100 mV, 1-15 mV and 0.5 mV values come from Binks et al., whose excitation voltage and units may differ from yours. They are a starting point, not a calibration.

## How it works

The sensor is averaged into 15-minute **steps**. Everything below is decided when a step finishes, by comparing it with the step before (its **rise**).

### Detecting wetting (leaf currently dry)

| Rise in one step | Result |
|---|---|
| `RAIN_RISE_MV` or more | **RAIN** |
| More than `DEW_MAX_RISE_MV`, less than `RAIN_RISE_MV` | **WET** (light rain, drizzle, fog; not in the paper) |
| `DEW_MIN_RISE_MV` to `DEW_MAX_RISE_MV`, `DEW_MIN_STEPS` times in a row | **DEW** |
| Anything else | Still dry |

While dry, the **baseline** follows the signal, which absorbs drift and dirt. When a rise begins, the baseline freezes at the reading just before it. That frozen value is the dry level for the whole wet period.

### Tracking a wet period

- **Rain on top:** a rise of `RAIN_RISE_MV` or more while already wet switches the state to **RAIN** and keeps the old baseline. This is how rain on a dewy leaf is handled (Binks et al. run two analyses with reversed priority; this is a single-pass simplification).
- **More water during rain:** a rise of `RAIN_RESURGE_MV` or more while in RAIN restarts the "time since rain" clock. This is an addition: the paper's threshold would miss rain that keeps falling on an already wet sensor.
- **Ending:** the leaf is dry again when a step falls below `WET_END_FACTOR` times the baseline. Dew must also have levelled off for `DEW_QUIET_STEPS` steps (the paper's "stable for 1.25 h" rule). The paper's dew end test uses the baseline itself; this sketch uses the same 110 % test for both, for simplicity.

### Start-up

At power-up the first reading becomes the baseline. If that reading is already above 110 % of `DRY_FALLBACK_MV`, the leaf is treated as wet and 350 mV is used as the baseline instead.

### Validation

Replaying the algorithm on `WET0002.CSV` (a real log, 8 hours, rain starting about 22:03 UTC) gave:

| Step ending (UTC) | Mean | Rise | Result |
|---|---|---|---|
| 22:07 | 377 mV | +17 mV | DRY -> WET |
| 22:22 | 542 mV | +165 mV | WET -> RAIN (baseline 353.5 mV) |
| 22:52 | 569 mV | +22 mV | rain resurge (clock restarted) |
| 00:22 | 618 mV | +30 mV | rain resurge (clock restarted) |

No false wetting in the dry first 40 minutes, and the leaf never dropped back below 110 % of baseline (about 389 mV) in the remaining 8 hours, so the state stayed RAIN. The log shows a slowly drying sensor at 466 mV even after 8 hours.

### Dew vs the air sensor

The BME680 is inside a sealed enclosure, so its temperature and humidity do not match the air at the leaves, and **no classification uses it**. It is logged for context only. Once a shielded sensor (e.g. SHT31) is mounted at canopy height, a humidity or dew-point check can be added.

This is a heuristic from one leaf sensor. It cannot distinguish every case (see Limitations).

## Output

### OLED (6 rows)

| Row | Shows | Example |
|---|---|---|
| 0 | State, warnings | `RAIN !GPS` |
| 1 | Leaf reading and baseline, in mV | `542 mV base 354` |
| 2 | Temp, RH, dew point | `14.2C 95% dp13.5` |
| 3 | Time wet or dry | `Wet for 1h 12m` |
| 4 | Time since rain was last detected | `Rain: 4h 10m ago` |
| 5 | Cumulative wet time since boot | `Wet total 6h 30m` |

Warnings on row 0: `!SD` means the SD card is missing or a write failed, `!GPS` means no valid GPS time yet.

### Serial, SD card and Bluetooth

Each sample produces one CSV row. Serial (debug builds only) prints the header at boot, and Bluetooth sends rows only (no header). Every boot starts a **new SD file** (`WET0001.CSV`, `WET0002.CSV`, ...) with the header as its first line; existing files are never appended to. In debug builds the chosen name is printed on Serial at startup (`# Logging to WET0003.CSV`). File names use the 8.3 format so they work with any SD library. Lines starting with `#` on Serial are status messages, not data.

## CSV reference

```
date,time,uptime_s,mv,state,baseline_mv,step_mv,step_rise_mv,wet_dur_s,since_wet_s,since_rain_s,wet_total_s,temp_c,rh_pct,dewpt_c
```

Rows come every 10 seconds, but the algorithm only decides every 15 minutes. `mv` is the live reading. `state`, `baseline_mv`, `step_mv` and `step_rise_mv` only change when a step finishes.

### Example rows

Dry leaf, GPS time available:

```
2026-10-13,21:30:03,480,353.6,DRY,353.8,353.5,-0.3,0,0,0,0,20.5,55.6,11.3
```

Three minutes after the rain was recognised:

```
2026-10-13,22:25:03,3790,545.2,RAIN,353.5,541.7,164.8,1090,0,190,1090,16.6,53.8,6.9
```

No GPS fix yet (date and time are empty):

```
,,42,353.8,DRY,353.8,353.8,0.0,0,0,0,0,20.5,55.8,11.4
```

### Columns

| # | Column | Unit | Description |
|---|---|---|---|
| 1 | `date` | `YYYY-MM-DD` | UTC date from the GPS. Empty until the GPS has a valid fix. |
| 2 | `time` | `HH:MM:SS` | UTC time from the GPS. Empty until the GPS has a valid fix. |
| 3 | `uptime_s` | s | Seconds since power-on. Always present. Wraps after about 49.7 days. |
| 4 | `mv` | mV | PHYTOS31 output at ADS1115 A0, average of 8 reads. The live measurement. |
| 5 | `state` | text | `DRY`, `DEW`, `RAIN`, or `WET` (wet, but not shaped like dew or heavy rain). Updated every 15 min. |
| 6 | `baseline_mv` | mV | The dry level. While dry it follows the signal; during a wet period it is frozen at the reading before wetting began. |
| 7 | `step_mv` | mV | Most recent finished 15-minute average. |
| 8 | `step_rise_mv` | mV | `step_mv` minus the average before it. Positive means wetting, negative means drying. |
| 9 | `wet_dur_s` | s | Length of the current wet period, back-dated to the start of the step (or dew run) that triggered it. 0 when dry. |
| 10 | `since_wet_s` | s | Time since the last wet period ended. 0 while wet or if the leaf has never been wet since boot. |
| 11 | `since_rain_s` | s | Time since rain was last detected (the rain rise, or a resurge of `RAIN_RESURGE_MV`). 0 means none has been seen since boot. |
| 12 | `wet_total_s` | s | Total wet time since boot, including the current wet period. |
| 13 | `temp_c` | degC | Air temperature from the BME680 (inside the enclosure). `nan` if the sensor is missing. |
| 14 | `rh_pct` | %RH | Relative humidity from the BME680. `nan` if the sensor is missing. |
| 15 | `dewpt_c` | degC | Dew point (Magnus formula) from the two values above. `nan` if the sensor is missing. |

### Reading the columns together

- **Detection lags by up to 15 minutes.** A wetting is only recognised when a step finishes, and `wet_dur_s` is back-dated to compensate. For exact onset times, look at the raw `mv` column.
- **Zeros are ambiguous.** `since_wet_s = 0` means "currently wet" or "never wet"; `since_rain_s = 0` means "no rain seen" (it counts up from 0 right after rain, so check `state`).
- **`since_rain_s` is not "time since the last drop".** It only restarts on a step-sized rise, so rain that falls steadily on an already wet sensor without raising the 15-minute average by 20 mV will not restart it.
- **Rain wetness can last for hours.** The state stays RAIN until the signal returns to within 10 % of the baseline, even long after the rain stopped. Use `since_rain_s` for how long ago it rained, and `state` for whether the sensor is still wet.
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

- **No real-time clock.** Durations and `uptime_s` restart at every reset, and so do the baseline and totals. Rows only carry real dates once the GPS has a fix.
- **One sensor cannot always separate dew, fog, drizzle and irrigation.** The `state` column is a best guess from the rate of rise.
- **Light rain can look like dew.** A rise of under 1 mV a step is ignored, and a run of small rises reads as dew, not rain.
- **Rain during a wet period only shows as a rise.** Steady rain on an already saturated sensor may raise the average by less than `RAIN_RESURGE_MV`, so it will not restart "time since rain".
- **A sensor is not a leaf.** Thermal mass, angle and exposure differ, and drops run off a tilted sensor, so onset and drying times will not match your canopy exactly. Mount it at leaf angle in the canopy.
- **Residue and contamination** (spray residue, dust, bird droppings) change the signal. The floating baseline absorbs slow drift, but clean the sensor periodically (Binks et al. cleaned theirs about every three months).
- **Thresholds are not tuned to your site.** They come from the paper and one log; compare against your own rain events.
- **CSV schema changes between versions.** Files from earlier sketch versions use different columns. New files are always written with the current header.

## Troubleshooting

| Symptom | Check |
|---|---|
| OLED shows `# ADS1115 not found` (and Serial repeats it when `DEBUG` is 1) | ADC wiring, power on 3V3_S, I2C address `0x48`. The sketch stops until the ADC is found. |
| `# RAK1906 not found` | Module seated in its slot. Logging continues with `nan` air values. |
| `# GPS not found` | Module and I2C wiring. Logging continues with blank date/time. |
| `# SD card not found` / `!SD` on OLED | Card seated and FAT-formatted. Logging continues on Bluetooth (and Serial in debug builds). |
| `# No free log file name on SD card` | All 9999 file names are used. Archive or delete old `WET*.CSV` files. |
| Blank or upside-down OLED | `U8G2_R2` rotates 180 degrees (use `U8G2_R0` for the other way). Confirm the I2C address and module. |
| State stuck on `DRY` during rain | Check `step_rise_mv` in the log. If rises are under `DEW_MAX_RISE_MV` the algorithm sees dew-sized changes; compare with raw `mv`. |
| State stuck on a wet state for hours | Normal after rain: it ends only when the signal returns near `baseline_mv`. Check `step_mv` against `baseline_mv` x 1.10. |
| `!GPS` never clears | GPS needs a clear sky view. Allow several minutes for a first fix. |

## Roadmap

- Shielded SHT31 at canopy height, then a humidity/dew-point check for DEW vs fog
- Real-time clock fallback so durations survive resets
- Test and document the OLED and SD paths
- Optional tipping-bucket rain gauge for a true "time since rain"

## References

- Binks, O., Carle, H., Coughlin, I., da Costa, A. L., & Meir, P. (2021). [Measuring the vertical profile of leaf wetness in a forest canopy](https://pmc.ncbi.nlm.nih.gov/articles/PMC8374337). *MethodsX* 8:101332.
- Kleb, M., Merkt, N., & Zorb, C. (2022). [New Aspects of In Situ Measurements for Downy Mildew Forecasting](https://www.mdpi.com/2223-7747/11/14/1807/htm). *Plants* 11(14):1807.
- Sentelhas, P. C., et al. (2005). [Spatial variability of leaf wetness duration in different crop canopies](https://ippc2.orst.edu/len/articles_canopy_conversion/sentelhas_etal_2005_full.pdf). *Int. J. Biometeorology* 49:363-370.

## License

Add a license of your choice (MIT is a common default for hardware/Arduino projects).
