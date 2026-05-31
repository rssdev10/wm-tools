# Screenshot Packet — Data Analysis Report

## Purpose

This report documents the ZeeWeii DSO3D12 **screenshot binary protocol**,
the format of the 2048-byte packet, and how the real captured data can be
used for accurate waveform drawing.

The implementation is in `dso3d12-parser/src/screenshot.rs`.
The capture CLI tool is `flasher/src/capture.rs`.

---

## Protocol Overview

### Trigger

Open the device **Gallery** (Screenshots view) and select a saved waveform.
The device automatically transmits a 2048-byte binary packet over the
USB-serial port at 115200 baud, 8N1.

This is entirely separate from the ASCII debug dump (triggered by long-press
Save in measurement view). The two protocols should **never be confused**:

| Property          | Debug dump (ASCII)      | Screenshot packet (binary) |
|-------------------|-------------------------|----------------------------|
| Size              | ~150–500 kB variable    | **2048 bytes fixed**       |
| Encoding          | ASCII decimal           | Binary little-endian       |
| Samples/channel   | 60 000 (full buffer)    | **300** (12 div × 25 px)   |
| Sample type       | One sample = one byte   | min/max per display pixel  |
| Metadata          | None                    | Rich instrument settings   |
| Trigger           | Long-press Save         | Gallery → select waveform  |

---

## Packet Layout (2048 bytes)

```
Offset    Size    Field
─────────────────────────────────────────────────────────────────
 0        300     CH1 waveform minimum ADC values (u8 × 300)
 300      300     CH1 waveform maximum ADC values (u8 × 300)
 600      300     CH2 waveform minimum ADC values (u8 × 300) *
 900      300     CH2 waveform maximum ADC values (u8 × 300) *
1200      340     Oscilloscope settings (DATA_DSO3D12 struct)
1540      100     CH1 auto-measurements (12 × i64 + 2 × i16 LE)
1640      100     CH2 auto-measurements *
1740      308     Reserved / padding
─────────────────────────────────────────────────────────────────
Total: 2048 bytes
* CH2 data is meaningless (but present) when CH2 is disabled.
```

### Why min + max?

The device downsamples the capture buffer (30 000–60 000 samples) to fit the
300-pixel display width. Each pixel covers multiple samples; the device stores
both the **minimum** and **maximum** sample value within that pixel's time
window. This is called an **envelope** representation, and it allows the display
to show waveform transitions that would otherwise alias away at lower resolution.

For a clean single-line trace: average `(min + max) / 2`.
For a "filled" oscilloscope-style trace: fill between `min` and `max`.

---

## ADC-to-Voltage Conversion

Raw ADC values are **inverted and zero-centred**: the ADC stores `255 - signal`.
The zero-volt reference is not necessarily at ADC code 128; it is shifted by the
device-calibrated `zero_volt_pixels` offset.

```
pixel_val  = 255 - raw          # uninvert
voltage(V) = (pixel_val - 128 - zero_volt_pixels) × (volt_scale_uv / 25) / 1e6
             × probe_multiplier
```

Where `volt_scale_uv` is the µV per **division** (from the VDIV table) and
division = 25 pixels. Probe multiplier: ×1=1, ×10=10, ×100=100.

---

## Key Settings Fields (byte offsets)

| Offset | Type  | Field                                | Notes                    |
|--------|-------|--------------------------------------|--------------------------|
| 1200   | u8    | run_state                            | 0=Stop 1=Run 2=Wait      |
| 1201   | u8    | system_state                         |                          |
| 1202   | u8    | ch2_enabled                          | 0=off 1=on               |
| 1204   | i64   | buffer_depth (samples)               | DSO3D12 only             |
| 1216   | u8    | timebase_index                       | 2=5ns … 30=10s           |
| 1220   | u64   | timebase_ns_per_div                  | ns per division          |
| 1228   | i64   | horiz_trigger_delay_ps               |                          |
| 1252   | u32   | sample_rate_hw_hz / 100              | DSO3D12 (×100 to get Hz) |
| 1256   | u8    | trigger_source                       | 0=CH1 1=CH2              |
| 1257   | u8    | trigger_mode                         | 0=Auto 1=Normal          |
| 1258   | u8    | trigger_edge                         | 0=Rise 1=Fall            |
| 1264   | u16   | trigger_level_adc                    | raw ADC count            |
| 1266   | u8    | ch1_vdiv_index                       | 1–13                     |
| 1268   | u32   | ch1_volt_scale_uv                    | µV per division          |
| 1272   | i32   | ch1_zero_volt_pixels                 | zero-ref pixel offset    |
| 1276   | i64   | ch1_zero_volt_uv                     | zero-ref in µV           |
| 1304   | u8    | ch1_coupling                         | 0=DC 1=AC                |
| 1305   | u8    | ch1_probe_mode                       | 0=×1 1=×10 2=×100        |
| 1328   | u8    | ch2_vdiv_index                       |                          |
| 1332   | u32   | ch2_volt_scale_uv                    |                          |
| 1336   | i32   | ch2_zero_volt_pixels                 |                          |
| 1392   | u64   | sample_rate_meas_hz                  | measurement sample rate  |
| 1400   | u32   | pre_trigger_samples                  |                          |
| 1404   | u32   | post_trigger_samples                 |                          |
| 1408   | u16   | display_start_col                    | viewport start           |
| 1410   | u16   | display_end_col                      | viewport end             |
| 1453   | u8    | cursors_x_enable                     |                          |
| 1454   | u8    | cursors_y_enable                     |                          |
| 1456   | i16   | cursor_x1 (pixels, +right of centre) |                          |
| 1458   | i16   | cursor_x2                            |                          |
| 1460   | i16   | cursor_y1 (pixels, signed)           |                          |
| 1462   | i16   | cursor_y2                            |                          |

---

## Measurement Block Layout (100 bytes, little-endian)

```
Bytes  0– 95  : 12 × i64 (8 bytes each)
  [0]  PkPk amplitude   (µV)
  [1]  Amplitude         (µV)
  [2]  Top               (µV)
  [3]  Base              (µV)
  [4]  Max               (µV)
  [5]  Min               (µV)
  [6]  Average           (µV)
  [7]  RMS               (µV)
  [8]  Positive width    (ps)
  [9]  Negative width    (ps)
  [10] Period            (ps)
  [11] Frequency         (µHz → divide by 1e6 to get Hz)

Bytes 96– 97  : i16  Positive duty cycle (units: 0.1%)
Bytes 98– 99  : i16  Negative duty cycle (units: 0.1%)
```

---

## Real Capture — Data Analysis

Captured 2025-05-31 from a DSO3D12 via `/dev/tty.usbserial-1110`.

### Instrument state

| Field              | Value                              |
|--------------------|------------------------------------|
| Device             | DSO3D12                            |
| Run state          | Run                                |
| Timebase           | 500 µs/div (index 17)              |
| Timebase raw       | 500 000 ns/div                     |
| Time/pixel         | 20 µs/px (500 µs ÷ 25 px/div)      |
| Total time span    | 6.00 ms (300 px × 20 µs/px)        |
| Buffer depth       | 30 000 samples                     |
| Pre-trigger        | 15 000 samples                     |
| Sample rate (meas) | 5.00 MSa/s                         |
| Sample rate (HW)   | 0 (field at offset 1252 is 0 here) |

### Channel 1

| Field              | Value                |
|--------------------|----------------------|
| V/div              | 1 V/div (index 7)    |
| Coupling           | AC                   |
| Probe              | ×10                  |
| Volt scale         | 100 000 µV/div       |
| Volt scale/pixel   | 4 000 µV/px (4 mV/px)|
| Zero offset (px)   | 20 px                |
| Zero offset (µV)   | 80 mV                |
| Display range      | ≈ −4.0 V … +4.0 V   |

**Waveform:** min=−2.22 V, max=+0.34 V, avg=−0.94 V

**Measurements:** 1 kHz frequency, 1.00 ms period, 50% duty cycle,
2.60 V peak-to-peak. This is a ~1 kHz square wave viewed in AC coupling
mode on a ×10 probe, with a small DC offset showing the AC coupling effect
(average is not 0 V because the probe/coupling haven't fully settled).

### Channel 2

| Field              | Value                  |
|--------------------|------------------------|
| V/div              | 200 mV/div (index 8)   |
| Coupling           | DC                     |
| Probe              | ×1                     |
| Volt scale/pixel   | 8 000 µV/px (8 mV/px)  |
| Zero offset (px)   | 0 px                   |
| Display range      | ≈ −0.8 V … +0.8 V     |

**Waveform:** min=−0.18 V, max=−0.17 V, avg=−0.176 V

**Measurements:** Essentially a DC offset of −176 mV. The very small
min/max spread (10 mV) is consistent with noise floor on a DC signal.

### Raw ADC bytes (CH1 min, first 16)

```
A1 A3 A3 A3 A3 A3 A3 A3 A3 A3 A3 A3 A3 A3 A3 A3
```

0xA3 = 163 decimal. Converted: `pixel = 255 - 163 = 92`. Voltage:
`(92 - 128 - 20) × 4000 µV/px × 10 (probe) = (-56) × 40 mV = -2.24 V`. ✓

---

## How This Data Drives Graph Drawing

### X axis

```
t[i] = i × (timebase_ns_per_div / 25) / 1e9   seconds
```

For this capture: `t[i] = i × 20 µs`, total span 0 … 5.98 ms.

### Y axis — voltage (per channel)

```
pixel_val = 255 - raw
voltage   = (pixel_val - 128 - zero_volt_pixels) × (volt_scale_uv / 25) / 1e6
            × 10^probe_mode
```

The `zero_volt_pixels` value calibrates the position of 0 V on the ADC scale
and must be subtracted — it represents the hardware DC offset of the channel.

### Waveform envelope (min/max shading)

Each pixel delivers a `(min_v, max_v)` range:
* **Simple trace**: draw the midpoint `(min_v + max_v) / 2`.
* **Envelope fill**: fill the area between `min_v` and `max_v` using a
  semi-transparent band — this is what the device's own screen does.
* **High-density mode**: draw both min and max as separate thin lines.

### Y-axis range for the graph widget

The full visible range from the device settings is:

```
half_range = volt_scale_uv × 4 / 1e6 × probe_multiplier   (V)
y_min = zero_volt_uv/1e6 - half_range
y_max = zero_volt_uv/1e6 + half_range
```

For this capture's CH1: `half_range = 100 000 × 4 / 1e6 × 10 = 4 V`,
so the display range is −4 V … +4 V.

### Auto-populating V/div and t/div in the GUI

After parsing a screenshot packet the GUI can auto-populate its measurement
panel inputs directly from device settings:

```
v_per_div = volt_scale_uv as f64 / 1_000_000.0 × probe_multiplier
t_per_div_ms = timebase_ns_per_div as f64 / 1_000_000.0   (ns → ms)
```

This eliminates the need for the user to manually enter scale values.

---

## Device Variant Field Differences

| Field              | DSO3D12 offset | DSO2512G V2/V1 offset |
|--------------------|----------------|-----------------------|
| buffer_depth       | 1204 (i64)     | 1208 (u64)            |
| sample_rate_hw     | 1252 (u32×100) | 1256 (u64×100)        |
| meas CH1 block     | 1540           | 1544                  |
| meas CH2 block     | 1640           | 1648                  |

---

## Observations & Known Issues

1. **`sample_rate_hw` reads 0 on this unit.** The field at offset 1252
   (DSO3D12) contains zeros in this firmware version. The measurement sample
   rate at offset 1392 (5.00 MSa/s) is correct and more reliable.

2. **CH1 avg ≠ 0 in AC mode.** The average of −0.94 V is because the
   waveform is captured near a DC transition — AC coupling capacitor has
   not fully settled, or this is expected asymmetric clipping.

3. **CH2 shows no measurements.** The measurement fields (freq, period, etc.)
   are zero for CH2. The device only computes auto-measurements for the active
   measurement channel. The raw waveform data (min/max ADC) is correct.

4. **Cursor data absent.** Both `cursors_x_enable` and `cursors_y_enable` are
   0 in this capture. Cursor fields will be non-zero when device cursors are
   enabled on the scope before saving.

5. **Waveform pixel count is always 300.** The 300-sample constraint is the
   display width — independent of buffer depth or timebase. Longer captures
   are downsampled by the device firmware to 300 pixels.
