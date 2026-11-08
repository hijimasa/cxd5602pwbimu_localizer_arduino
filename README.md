# Spresense IMU Localizer

English | [日本語](./README_jp.md)

This repository contains an Arduino IDE project that performs inertial localization with the Spresense IMU add-on board. The Spresense multicore system is split into an **I/O core (MainCore)** and a **processing core (SubCore1)**. Attitude, velocity and position are estimated with the **Fusion** AHRS library, a zero-velocity update (ZUPT) and trapezoidal integration.

The algorithm itself is implemented as Arduino-independent C++ modules (`SubCore1/src/localizer/`), so it can be unit-tested and regression-tested on a host PC with synthetic IMU data.

## Overview

The system reads raw IMU data (acceleration, angular rate, temperature) at 1920 Hz and processes it on two cores:

- **MainCore:** IMU acquisition, writes into a shared ring buffer, serial output
- **SubCore1:** initial alignment, Gaussian filter, earth-rate compensation, Fusion AHRS, ZUPT, bias learning, velocity/position integration

## Architecture

```
┌────────────────────┐   shared-memory   ┌──────────────────────────────────┐
│      MainCore      │   ring buffer     │             SubCore1             │
│  IMU read 1920 Hz  │ ───────────────▶ │ alignment → Gaussian →           │
│  serial out 30 Hz  │ ◀─────────────── │ earth-rate comp. → Fusion AHRS → │
│                    │  position (30 Hz) │ ZUPT → bias learning → integ.    │
└────────────────────┘  stats (0.2 Hz)   └──────────────────────────────────┘
```

IMU samples cross the core boundary through a 256-entry ring buffer (about 133 ms) owned by MainCore. SubCore1 keeps its own read position and drains every unread sample, even if a notification message was dropped. Data is lost only if SubCore1 falls behind by 255 samples or more, and every lost sample is counted as an "overrun" in the statistics.

## Features

- **Lossless data path:** ring buffer instead of a single overwritten struct; losses are detected and counted
- **High-speed acquisition:** 1920 Hz, the full performance of the IMU
- **Gyro-compass initial alignment:** Up from the mean accelerometer vector, North from the horizontal component of the mean angular rate; the initial quaternion matches the Fusion NWU convention. With the gyro bias from the six-orientation calibration removed first, the heading is true north instead of arbitrary
- **Initial gyro offset and earth-rate compensation:** the mean angular rate minus the expected earth rotation seeds the gyro offset; while running, the earth rotation is continuously removed using the current attitude
- **Heading observer (gyro-compass re-estimation at rest):** with the gyro preset, every rest is used to re-fit the gyro bias and the heading from the earth rotation; the attitude and the path are turned slowly towards the result. Rests at two or more headings separate a drifted bias from the earth rate, so a wrong start-up heading (for example the East/West mirror image a bias error larger than the horizontal earth rate produces) is corrected after a turn and a stop
- **Fusion AHRS:** acceleration rejection and gravity removal
- **ZUPT (variance + mean):** besides the signal variance, the window mean of acceleration and angular rate is checked, so a smooth acceleration is not mistaken for rest; while the estimated speed is large the confirmation period is extended to survive short constant-velocity phases
- **AHRS gain scheduling:** high gain while stationary for fast convergence, low gain during motion so that linear acceleration does not tilt the attitude
- **Accelerometer bias learning** while stationary
- **Trapezoidal integration with retroactive ZUPT correction:** the velocity left at a confirmed ZUPT is the error accumulated over the preceding motion segment; assuming linear growth it is removed from the position
- **Host-side unit tests** with synthetic IMU regression scenarios

## Hardware Requirements

- **Sony Spresense main board** (CXD5602)
- **Spresense IMU add-on board** (CXD5602PWBIMU)

## Software Requirements

- **Arduino IDE** (1.8.x or 2.x) or **arduino-cli**
- **Spresense Arduino board package** (3.0.0 or later)
- g++ (C++17) and make for the tests

## Getting Started

### Installation

1. **Clone the repository:**

   ```bash
   git clone https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino.git
   cd cxd5602pwbimu_localizer_arduino
   git submodule update --init
   ```

   The Fusion library is a submodule, referenced from `SubCore1/src/Fusion` through a symbolic link. The headers shared by both cores (`shared_types.h`, `vec_math.h`, `accel_bias_fit.h`, `gyro_bias_fit.h`, `localizer_config.h`) are symbolic links as well.

   **Windows:** git creates these links only when symbolic links are enabled; otherwise it checks out small text files containing the link targets and the build fails with missing-header errors. Turn on *Developer Mode* (Settings > System > For developers), which allows creating symbolic links without administrator rights, and clone with links enabled:

   ```powershell
   git clone -c core.symlinks=true --recursive https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino.git
   ```

   With real links arduino-cli and the Arduino IDE build and upload natively (the serial port is `COMx` for the IDE and the Python tools). Check the link to the submodule, which is the one that matters: `cmd /c dir SubCore1\src` must list `Fusion` as `<SYMLINKD>` (a directory link). Git for Windows turns a link into a *file* link when its target does not exist yet, and with `--recursive` the submodule is populated after the links are created; `.gitattributes` marks `SubCore1/src/Fusion` as `symlink=dir` to prevent this, but a clone made before that attribute existed, or without `core.symlinks=true`, shows `<SYMLINK>` instead and the build fails with `../Fusion/Fusion.h: No such file or directory`. Repair it by recreating the link now that the submodule exists:

   ```powershell
   git config core.symlinks true
   Remove-Item SubCore1\src\Fusion
   git checkout -- SubCore1/src/Fusion
   ```

   If the header links in the repository root are text files too, run `git checkout -- .` after setting `core.symlinks` (or clone again). The host tests (`make`, g++) need WSL or MSYS2; WSL can also build and upload, but then the USB-serial bridge has to be attached to WSL with usbipd-win.

2. **Install the Spresense board package:**

   - Open the Arduino IDE
   - **File** > **Preferences**
   - Add the following to "Additional Boards Manager URLs":
     ```
     https://github.com/sonydevworld/spresense-arduino-compatible/releases/download/generic/package_spresense_index.json
     ```
   - **Tools** > **Board** > **Boards Manager**
   - Search for "Spresense" and install the package

### Build and Upload (multicore)

Upload SubCore1 **before** MainCore.

1. **SubCore1 (all processing):**
   - Open `SubCore1/SubCore1.ino`
   - **Tools** > **Core** > **"SubCore 1"**
   - Click **Upload**

2. **MainCore (IMU acquisition and output):**
   - Open `cxd5602pwbimu_localizer_arduino.ino`
   - **Tools** > **Core** > **"MainCore"**
   - Click **Upload**

With arduino-cli:

```bash
arduino-cli compile --fqbn SPRESENSE:spresense:spresense:Core=Sub1 SubCore1
arduino-cli compile --fqbn SPRESENSE:spresense:spresense:Core=Main .
```

### Running

1. Open the serial monitor after uploading both cores
2. Set the baud rate to **115200**
3. Keep the sensor completely still for the first **15 seconds** (initial attitude, gyro offset and accelerometer bias calibration)
4. Position data is then printed at 30 Hz

### Output Format

Each line contains 18 comma-separated values. Except for the timestamp, every value is the IEEE754 bit pattern of a float printed as 8 hex digits:
```
timestamp, temp, gx, gy, gz, ax, ay, az, qw, qx, qy, qz, vx, vy, vz, px, py, pz
```

- `gx..gz`: offset-corrected angular rate [rad/s], sensor frame
- `ax..az`: gravity-removed, bias-corrected acceleration [m/s²], Earth frame (NWU)
- `qw..qz`: attitude quaternion (sensor relative to Earth, NWU)
- `vx..vz`, `px..pz`: velocity [m/s] and position [m], Earth frame

Building with `VERBOSE_OUTPUT` defined adds a statistics block every 5 s with the processing rate, ring-buffer overruns, dt anomalies and the estimated biases.

## Tests

The algorithm can be tested on a host PC:

```bash
cd test
make                      # build and run all tests
make run ARGS=localizer   # run only tests whose name contains "localizer"
make tools-test           # unit tests of the Python tools (parser, projection, dashboard widgets)
```

The suite covers:

- quaternion conversions agree with the Fusion library convention
- initial alignment recovers random attitudes (including a regression test for tilted start-up)
- unit tests of the Gaussian filter, ZUPT detector and integrator (the previous implementation is kept as a reference for equivalence checks)
- ring buffer: no loss within capacity, overrun detection, concurrent producer/consumer
- end-to-end regression scenarios with synthetic IMU data (noise, bias, earth rotation): stationary, tilted start, rotation, translation, timestamp wrap-around, dt on lost samples
- six-orientation fits: accelerometer sphere fit and gyro earth-rotation fit (synthetic faces with random headings, the real board's means, degenerate orientation sets), gyro preset restores north and cannot cause drift when it is off, raw-log frame parsing
- heading observer: single rest reproduces the gyro-compass, East/West rests separate a y-axis bias error from the earth rate, the vertical bias is observable at one heading, rests contaminated by a turn are dropped, ageing of old rests; end to end, a 180° wrong start-up heading is corrected to 2.5° after a turn and a rest (Earth-frame path error 3 cm instead of 97 cm), a 180 s rest refines the heading, the observer is idle without a preset

Every scenario prints its current performance figures as `metric` lines. Tighten the bounds as the algorithm improves; never loosen them.

## Accelerometer and gyro bias calibration (six orientations)

The stationary calibration at start-up can only observe the accelerometer bias along gravity; the components perpendicular to it are absorbed into the attitude and show up as horizontal errors once the sensor is tilted a lot or carried by hand. The gyro bias is the other limit: it is usually larger than the horizontal earth rate (0.0034 °/s in Tokyo), which makes the gyro-compass heading arbitrary. The normal build offers a guided six-orientation calibration over the serial port that determines both; the result is stored in the SPI flash and used automatically from the next boot on.

**Let the board warm up first.** The gyro bias drifts with temperature (about −0.00035 °/s per °C on the z axis of this board, i.e. 0.001 °/s over 3 °C, a third of the horizontal earth rate). The IMU temperature settles about 30 minutes after power-on, so leave the board running for 30 minutes before calibrating, and calibrate at the temperature it will be used at.

The bundled script walks through the whole procedure:

```bash
python3 tools/calibrate_accel_bias.py /dev/ttyUSB0          # interactive six-orientation calibration
python3 tools/calibrate_accel_bias.py /dev/ttyUSB0 --show   # print the stored preset
python3 tools/calibrate_accel_bias.py /dev/ttyUSB0 --erase  # erase the stored preset
```

Doing it by hand from a serial monitor:

1. Send `C` from the serial monitor (115200 baud). Position output pauses and the first orientation is requested
2. Put the sensor in the requested orientation (+Z up = board flat, −Z up = upside down, ±X / ±Y up = board standing on an edge), keep it still and send `Y`. Ten seconds are measured; if the sensor moved you are asked to repeat that orientation
3. After six orientations the accelerometer bias, the gyro bias, their residuals and the vertical earth rate seen by the gyro are printed and saved. SubCore1 restarts its 15 s stationary calibration with the new values, so keep the sensor still

A few degrees of misalignment per orientation do not matter (six roughly orthogonal directions are enough). Other commands: `P` print the stored preset, `X` erase it, `R` restart the localizer (15 s still), `Q` abort the calibration. The raw-logging build ignores commands but writes the stored preset at the start of the recording so that the replay applies the same values (`--no-preset`, `--preset x y z` in m/s², `--gyro-preset x y z` in °/s override them).

How the gyro bias is found: at rest the gyro measures the earth rotation plus the bias. The horizontal part of the earth rotation depends on the heading of each orientation, which is unknown, but its component along gravity is the same everywhere: Ω·sin(latitude), 0.0024 °/s in Tokyo. The gravity direction of each orientation is known from the accelerometer mean (after the sphere-fit bias is removed), so each orientation gives one linear equation for the bias, and the six faces give a well-determined least-squares solution; between the two faces of each axis the latitude term even cancels. As a check the vertical earth rate is also solved as a free parameter: it should come out near the expected value for the configured latitude (8.8 °/h in Tokyo; the real board gives 8.8 °/h), which proves that the gyro really sees the earth rotation. The result is only used to find north at start-up; the run-time gyro offset is still taken from the 15 s stationary mean, so a preset that no longer fits (temperature, turn-on repeatability) can degrade the heading but never adds drift at rest.

Typical result for this board: accelerometer z ≈ +0.04 m/s², y ≈ −0.012 m/s²; gyro ≈ (−0.005, −0.003, −0.0235) °/s at 37 °C.

## Recording and replaying device data

Building MainCore with `RAW_LOG_OUTPUT` writes every IMU sample (1920 Hz) and the on-device position estimate (30 Hz) as binary frames to the serial port at 921600 baud. The recording can be replayed on the host through the same algorithm.

```bash
# build and upload the recording MainCore (SubCore1 stays the same)
arduino-cli compile --upload -p /dev/ttyUSB0 \
  --build-property "build.extra_flags=-DRAW_LOG_OUTPUT" \
  --fqbn SPRESENSE:spresense:spresense:Core=Main .

# record 60 s (opening the port resets the board: the 15 s calibration is included)
python3 tools/record_raw.py /dev/ttyUSB0 60 logs/desk_1m.bin

# replay on the host (--csv writes the 30 Hz estimate; features can be disabled for comparison)
cd test && make replay
./out/replay ../logs/desk_1m.bin --csv desk_1m.csv
./out/replay ../logs/desk_1m.bin --no-retro
```

The calibration takes 15 s of samples after the board has booted, i.e. it ends about 18 s after the script was started. The script prints "calibration complete" once it has received the calibration samples, and the board shows LED0 while calibrating and LED1 while running. Moving during the calibration corrupts the initial attitude and gyro offset, so wait for that cue (a contaminated recording can sometimes be rescued by replaying with a shorter calibration, e.g. `--calib-seconds 13`).

The replay prints the maximum deviation from the device's own estimate; since both run the same code it is normally 0 for the position and below 1e-5 for the quaternion (a device running an older firmware shows a difference).

`tools/analyze_replay.py` tabulates a replay CSV into rest/motion segments with their displacements and the attitude change. The recordings in `test/data/` are also used by the regression tests (`test_recordings.cpp`), which check closure error and displacement ranges.

```bash
python3 tools/analyze_replay.py desk_1m.csv
```

The recording firmware stays silent for 2 s after boot so that the flash writer can synchronise with the boot ROM. Reading the 921600-baud stream at another baud rate can nevertheless lock up the host's USB-serial bridge (CP210x): nothing is received any more and an upload hangs in `flash_writer` waiting for the prompt. Kill `flash_writer`, run `python3 tools/usb_reset_cp210x.py` (USB-level reset, no unplugging needed) and retry the upload.

## Project Structure

```
cxd5602pwbimu_localizer_arduino/
├── cxd5602pwbimu_localizer_arduino.ino  # MainCore: IMU acquisition, serial output, six-orientation calibration
├── shared_types.h                        # inter-core structs and ring buffer
├── accel_bias_fit.h, gyro_bias_fit.h,    # links into SubCore1/src/localizer/ (used by MainCore's calibration)
├── vec_math.h, localizer_config.h
├── SubCore1/
│   ├── SubCore1.ino                      # SubCore1: runs the pipeline, sends results
│   ├── shared_types.h -> ../shared_types.h
│   └── src/
│       ├── Fusion -> ../../Fusion/Fusion # Fusion library (submodule)
│       └── localizer/                    # Arduino-independent algorithm
│           ├── localizer_config.h        # all parameters
│           ├── vec_math.h                # vectors and quaternions
│           ├── initial_alignment.*       # initial attitude / offset / bias
│           ├── accel_bias_fit.h          # six-orientation sphere fit (accelerometer)
│           ├── gyro_bias_fit.h           # six-orientation earth-rotation fit (gyro)
│           ├── heading_observer.*        # gyro-compass re-estimation of heading and bias at rest
│           ├── gaussian_filter.*         # causal Gaussian filter
│           ├── zupt_detector.*           # variance based ZUPT
│           ├── velocity_integrator.*     # trapezoidal integration
│           └── localizer.*               # pipeline
├── test/                                 # host unit tests (make)
│   ├── imu_simulator.h                   # synthetic IMU generator
│   ├── raw_log.h                         # recording file parser
│   ├── replay.cpp                        # replay device recordings (make replay)
│   ├── data/                             # device recordings used by the regression tests
│   └── test_*.cpp
├── tools/
│   ├── record_raw.py                     # record raw device data
│   ├── analyze_replay.py                 # segment analysis of a replay CSV
│   ├── imu_stream.py                     # serial / file / CSV input shared by the viewers
│   ├── imu_monitor.py                    # terminal dashboard
│   ├── imu_viewer.py                     # tkinter 3D viewer (numba kernels)
│   ├── test_imu_tools.py                 # unit tests of the tools (python3 -m unittest)
│   ├── calibrate_accel_bias.py           # interactive six-orientation calibration
│   └── usb_reset_cp210x.py               # recover a locked-up USB-serial bridge
└── Fusion/                               # Fusion library (submodule)
```

## Algorithm Details

### Initial alignment (15 s, stationary)

1. Average the accelerometer and gyroscope readings
2. Up = direction of the mean acceleration, North = horizontal component of the mean angular rate (after removing the gyro bias preset when one is stored), West = Up × North; the initial quaternion is built from this NWU triad
3. The earth rotation vector for the configured latitude is rotated into the sensor frame and subtracted from the mean angular rate; the remainder is the gyro offset
4. The expected gravity vector is subtracted from the mean acceleration to obtain the accelerometer bias (the component perpendicular to gravity is unobservable at a single orientation, which is why the six-orientation preset is subtracted beforehand)

### Per-sample pipeline

1. **dt:** from the IMU timestamp difference (32-bit wrap safe). Values above 1.5× nominal are counted as anomalies, values above 10× are clamped
2. **Gaussian filter:** causal, σ = 2 samples, largest weight on the current sample
3. **Earth-rate compensation:** the earth rotation vector is rotated into the sensor frame with the current attitude and subtracted from the angular rate
4. **Gyro offset tracking:** FusionBias (offset updated only after 5 s below 3 °/s), seeded with the alignment result. With the gyro preset the offset is the heading observer's bias estimate instead (step 11): FusionBias would absorb the residual of a wrong heading at rest and freeze the error
5. **AHRS update:** Fusion (acceleration rejection 10°, gain scheduled in 7b)
6. **Gravity removal and bias correction** in the sensor frame, then rotation into the Earth frame
7. **ZUPT:** stationary when, over a 16-sample window, the variance is below the thresholds and the mean acceleration is below 0.1 m/s² and the mean angular rate below 2 °/s, for 96 consecutive samples (50 ms). While the estimated speed exceeds 0.1 m/s, 1 s of continuous stillness is required instead (a short constant-velocity phase in the middle of a move must not be taken for a stop). Leaving is immediate
7b. **Gain scheduling:** confirmed stationary → 0.5; moving (high variance, or large mean acceleration/rotation) → 0.02; low variance and low mean but not yet confirmed → 0.1. A "low variance but large mean acceleration" state lasting more than 2 s is assumed to be an attitude error at rest and falls back to 0.1
8. **Bias learning:** while stationary, the residual acceleration is learned with a 1 s time constant
9. **Integration:** trapezoidal velocity and position; velocity is reset to zero while stationary
10. **Retroactive ZUPT correction:** when a ZUPT is confirmed at the end of a motion segment of duration T, the remaining velocity v_err is error and is removed from the position: during the confirmation delay t_still the error velocity was constant (v_err × t_still), before that it is modelled as growing linearly (v_err × (T − t_still) / 2). To keep the output continuous the correction is spread linearly over the second after the ZUPT instead of being applied as a step (a further ZUPT during that time merges its correction). Segments in which the velocity limit was hit, or longer than 60 s, are not corrected
11. **Heading observer** (only with the gyro preset): at rest the gyroscope measures bias + earth rotation in the sensor frame. The mean of every rest of at least 2 s (the first 0.3 s and the last 0.3 s are left out because the ZUPT detector lags the start and the end of a motion) is stored with the AHRS attitude at that time; the alignment is the first such rest. Every 0.5 s at rest the last 8 rests plus the one in progress are fitted by weighted least squares for the bias (3 unknowns) and the yaw error of the AHRS frame (cos/sin, held on the unit circle), with the preset as a prior (σ = 0.002 °/s) and each rest weighted by its length (noise of a mean ∝ 1/√T, capped at 60 s) and discounted with age (600 s constant, temperature drift). At a single heading the bias and the heading cannot be told apart and the prior decides: the result is the gyro-compass of the preset, as at start-up. Rests at different headings make the bias observable (it is fixed to the sensor, the earth rate turns with the heading): East then West, for example, gives the y-axis bias exactly. The yaw error is applied as a rotation about Up of the attitude and of the integrated velocity and position (the path stays rigid, the frame becomes true), with a 5 s time constant and a 15 °/s slew limit. The fit reports the earth rate it sees (15.04 °/h in total, 8.77 °/h vertical in Tokyo); a rest whose mean is more than 0.03 °/s from the bias was not a rest and is rejected

### Current performance on synthetic data (`metric` lines of `cd test && make`)

| Scenario | Position error |
|---|---|
| 0.3 m in 0.5 s (abrupt) | 0.6 mm |
| 1 m in 2 s (smooth) | 3.5 mm |
| 2 m in 3 s | 8.3 mm |
| 1 m in 4 s (slow, peak 0.36 m/s²) | 2.0 cm |
| 90° turn in place | 1.5 cm, attitude error 0.03° |
| Start-up heading 176° wrong (y-axis preset error 0.006 °/s), 180° turn, 42 s rest, 0.5 m move | heading 2.5°, Earth-frame position error 3 cm (97 cm without the observer) |

### Results on device recordings (`test/data/`, round trips back to the start)

| Test | Estimated displacement | Closure error |
|---|---|---|
| 30 cm on the desk and back (fast, up to 0.57 m/s) | 0.309 / 0.302 m | 4.5 mm |
| 30 cm on the desk and back (2.5 s each way, up to 0.20 m/s) | 0.302 / 0.289 m | 3.3 mm |
| 30 cm on the desk and back (slow, 4.5 s each way, up to 0.12 m/s) | 0.306 / 0.316 m (with correction) | 1.2 cm |
| 30 cm on the desk and back (9 s each way, up to 0.05 m/s) | 0.326 / 0.32 m (with correction) | 9.5 mm |
| Lift, carry 70 cm horizontally and 45 cm down while rotating ~90°, place, carry back | 0.75 m horizontal, 0.44 m down (with correction; 0.59 m horizontal without) | 1.0 cm |
| Stationary 60 s (level, upside down, 30° tilt) | zero position drift | attitude change 0.000° |

With both the retroactive correction and the gain scheduling disabled the closure error of the carry test is 33 cm.

### Known limitations (next improvement targets)

- **Constant-velocity motion longer than 1 s is indistinguishable from rest.** This is inherent to a stand-alone IMU; shorter coasting phases are handled by the speed-dependent confirmation time
- **Slow motion** with accelerations around 0.1 m/s² sits at the mean-acceleration threshold and comes out short
- **Gyro-compass accuracy depends on the gyro bias.** Without the six-orientation gyro preset the bias of this board (0.006 °/s horizontal) exceeds the horizontal earth rate (about 0.0034 °/s in Tokyo) and the initial heading is arbitrary. With the preset the heading noise of a 15 s alignment is about 5°, and a temperature change of a few °C since the calibration adds a comparable error; a bias error above the horizontal earth rate even returns the mirror-image heading (x East and x West both read East). The heading observer removes the part of this that rests at different headings can observe: after a turn and a stop the bias is re-fitted and the heading corrected; at a single heading the observer can only average the noise down (about 2° after 60 s). Relative positioning is unaffected either way; the tests evaluate position error in the initial-heading-aligned frame unless the observer is under test. Modelling the temperature dependence of the bias is the next step
- **Thresholds have not been tuned on real motion data.** The synthetic data includes noise, bias and earth rotation but not the real vibration signature. Recordings from the device can be replayed on the host (see below) to close this gap

## Configuration

Main parameters in `SubCore1/src/localizer/localizer_config.h`:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `LOCALIZER_SAMPLE_RATE` | 1920 Hz | IMU sample rate |
| `LOCALIZER_GRAVITY` | 9.7975 m/s² | local gravity (Tokyo) |
| `LOCALIZER_LATITUDE_DEG` | 35.68° | latitude for earth-rate compensation (Tokyo) |
| `LOCALIZER_CALIBRATION_SECONDS` | 15 s | stationary calibration period at start-up |
| `FUSION_GAIN_STATIONARY` / `SETTLING` / `MOTION` | 0.5 / 0.1 / 0.02 | AHRS gain per state |
| `FUSION_ACCEL_REJECTION` | 10.0° | acceleration rejection threshold |
| `ZUPT_ACCEL_VARIANCE_THRESHOLD` | 0.004 (m/s²)² | ZUPT acceleration variance threshold |
| `ZUPT_GYRO_VARIANCE_THRESHOLD` | 0.00006 (rad/s)² | ZUPT angular-rate variance threshold |
| `ZUPT_ACCEL_MEAN_THRESHOLD` | 0.1 m/s² | ZUPT mean-acceleration threshold |
| `ZUPT_GYRO_MEAN_THRESHOLD` | 2 °/s | ZUPT mean angular-rate threshold |
| `ZUPT_SLOW_CONFIRM_SPEED` / `SAMPLES` | 0.1 m/s / 1 s | confirmation period while the estimated speed is large |
| `ZUPT_RETROACTIVE_CORRECTION` | 1 | enable the retroactive ZUPT position correction |
| `ZUPT_CORRECTION_BLEND_TIME` | 1.0 s | time over which the correction is spread (0 = instant) |
| `HEADING_OBSERVER_ENABLED` | 1 | gyro-compass re-estimation at rest (needs the gyro preset) |
| `HEADING_OBSERVER_PRESET_SIGMA` | 0.002 °/s | how far the preset may be off (prior of the bias fit) |
| `HEADING_OBSERVER_MIN_PERIOD` / `MAX_PERIOD` / `MEMORY` | 2 s / 60 s / 600 s | shortest rest used, length beyond which a rest is not trusted more, age constant |
| `HEADING_OBSERVER_CORRECTION_TIME` / `MAX_RATE` | 5 s / 15 °/s | how the heading correction is applied |
| `IMU_RING_SIZE` (shared_types.h) | 256 | inter-core ring buffer length |
| `IMU_NOTIFY_INTERVAL` (shared_types.h) | 8 | samples between wake-up notifications to SubCore1 (the receive FIFO has only 8 buffers) |

## Visualization

Two stand-alone tools in `tools/` show the serial output of the normal build live. Both take the same inputs: a serial port (`--port /dev/ttyUSB0`, `COM3` on Windows), a text file of saved serial lines (`--file`) or a replay CSV from `test/replay --csv` (`--csv`), the files being replayed at their recorded pace (`--speed 5`, `--fast`). Opening the port resets the board, so the first 15 s are the stationary calibration and nothing is shown until the first estimate arrives. Dependencies: `pip install pyserial numpy` (and `numba` for the viewer, optional).

### Terminal monitor

```bash
python3 tools/imu_monitor.py --port /dev/ttyUSB0
python3 tools/imu_monitor.py --csv replay.csv --speed 5
python3 tools/imu_monitor.py --port /dev/ttyUSB0 --dump > log.csv   # decoded CSV instead of the dashboard
```

A dashboard redrawn in place with ANSI escape codes (no curses, works in Windows Terminal): attitude, angular rate, acceleration, velocity and position with bars, the ZUPT state (the velocity is exactly zero while stationary), sparklines of |a| and |ω| over the last 20 s, an ASCII top view of the path (north up, west left) and the last board messages. `--dump` prints one decoded line per sample instead, for logging or piping.

### 3D viewer (GUI)

```bash
python3 tools/imu_viewer.py --port /dev/ttyUSB0
python3 tools/imu_viewer.py --csv replay.csv --speed 5
```

A tkinter window (no Qt or OpenGL needed) with a software-rendered 3D view: the **origin frame** (N red, W green, U blue), the **current sensor pose** as a box (long side along body Y, as on the board) with its body axes, the path since start and a drop line to the ground grid, next to the numbers (position, velocity, attitude, angular rate, acceleration), the pose **relative to the origin frame** and strip charts of acceleration and angular rate. Mouse: left drag orbits, right drag pans, wheel zooms. Keys: `r` frames the whole path, `f` follows the sensor, `z` moves the origin frame to the current pose (the relative pose then counts from there), `Z` resets it, `c` clears the path, space pauses, `q` quits.

The per-frame numerics (rotation of the geometry, perspective projection of up to 20 000 path points, reduction of the path to the points that are at least one pixel apart on screen) run in numba-compiled kernels when numba is installed and fall back to numpy otherwise; the kernels are compiled once and cached under `tools/__pycache__`. The canvas keeps its items and only updates their coordinates, so a frame with a 20 000-point path takes a few milliseconds on a laptop (the title line shows the render time).

### ROS 2 integration

For real-time visualization and integration with the ROS ecosystem use the ROS 2 package:

**[cxd5602pwbimu_localizer_node](https://github.com/hijimasa/cxd5602pwbimu_localizer_node)**

The node reads the serial output of this project and converts it into standard ROS messages:
- `sensor_msgs/Imu` – IMU data (angular rate, linear acceleration, attitude)
- `geometry_msgs/Pose` – position and attitude
- TF transforms for RViz

#### Quick start

```bash
cd ~/ros2_ws/src
git clone https://github.com/hijimasa/cxd5602pwbimu_localizer_node.git
cd ~/ros2_ws
colcon build --packages-select cxd5602pwbimu_localizer_node
source install/setup.bash

ros2 run cxd5602pwbimu_localizer_node localizer_node --ros-args -p serial_port:=/dev/ttyUSB0
```

Add TF and Pose displays in RViz to visualize the sensor data.

## Troubleshooting

1. **No output:**
   - Make sure SubCore1 has been uploaded
   - Nothing is printed during the 15 s calibration after boot

2. **Position drift while stationary:**
   - Keep the sensor completely still during the initial calibration
   - Check with `VERBOSE_OUTPUT` that overruns and dt_anomalies stay at 0

3. **Compile errors:**
   - Run `git submodule update --init` (`SubCore1/src/Fusion` points into the submodule)
   - `../Fusion/Fusion.h: No such file or directory` on Windows: `SubCore1/src/Fusion` is not a directory link. `cmd /c dir SubCore1\src` must show `<SYMLINKD>`; if it shows `<SYMLINK>` or a plain file, recreate the link as described under Installation
   - Check that the Spresense board package is installed
   - Check that the correct core is selected in the Tools menu

## Contributing

Contributions are welcome. Fork the repository and open a pull request; for larger changes please open an issue first. Run `cd test && make` before submitting.

## License

MIT License, see `LICENSE`.

---

For questions or problems open a GitHub issue or contact the repository maintainer.
