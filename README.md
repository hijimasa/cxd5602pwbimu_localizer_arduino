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
- **Gyro-compass initial alignment:** Up from the mean accelerometer vector, North from the horizontal component of the mean angular rate; the initial quaternion matches the Fusion NWU convention
- **Initial gyro offset and earth-rate compensation:** the mean angular rate minus the expected earth rotation seeds the gyro offset; while running, the earth rotation is continuously removed using the current attitude
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

   The Fusion library is a submodule, referenced from `SubCore1/src/Fusion` through a symbolic link.

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
```

The suite covers:

- quaternion conversions agree with the Fusion library convention
- initial alignment recovers random attitudes (including a regression test for tilted start-up)
- unit tests of the Gaussian filter, ZUPT detector and integrator (the previous implementation is kept as a reference for equivalence checks)
- ring buffer: no loss within capacity, overrun detection, concurrent producer/consumer
- end-to-end regression scenarios with synthetic IMU data (noise, bias, earth rotation): stationary, tilted start, rotation, translation, timestamp wrap-around, dt on lost samples

Every scenario prints its current performance figures as `metric` lines. Tighten the bounds as the algorithm improves; never loosen them.

## Project Structure

```
cxd5602pwbimu_localizer_arduino/
├── cxd5602pwbimu_localizer_arduino.ino  # MainCore: IMU acquisition & serial output
├── shared_types.h                        # inter-core structs and ring buffer
├── SubCore1/
│   ├── SubCore1.ino                      # SubCore1: runs the pipeline, sends results
│   ├── shared_types.h -> ../shared_types.h
│   └── src/
│       ├── Fusion -> ../../Fusion/Fusion # Fusion library (submodule)
│       └── localizer/                    # Arduino-independent algorithm
│           ├── localizer_config.h        # all parameters
│           ├── vec_math.h                # vectors and quaternions
│           ├── initial_alignment.*       # initial attitude / offset / bias
│           ├── gaussian_filter.*         # causal Gaussian filter
│           ├── zupt_detector.*           # variance based ZUPT
│           ├── velocity_integrator.*     # trapezoidal integration
│           └── localizer.*               # pipeline
├── test/                                 # host unit tests (make)
│   ├── imu_simulator.h                   # synthetic IMU generator
│   └── test_*.cpp
└── Fusion/                               # Fusion library (submodule)
```

## Algorithm Details

### Initial alignment (15 s, stationary)

1. Average the accelerometer and gyroscope readings
2. Up = direction of the mean acceleration, North = horizontal component of the mean angular rate, West = Up × North; the initial quaternion is built from this NWU triad
3. The earth rotation vector for the configured latitude is rotated into the sensor frame and subtracted from the mean angular rate; the remainder is the gyro offset
4. The expected gravity vector is subtracted from the mean acceleration to obtain the accelerometer bias (the component perpendicular to gravity is unobservable at a single orientation)

### Per-sample pipeline

1. **dt:** from the IMU timestamp difference (32-bit wrap safe). Values above 1.5× nominal are counted as anomalies, values above 10× are clamped
2. **Gaussian filter:** causal, σ = 2 samples, largest weight on the current sample
3. **Earth-rate compensation:** the earth rotation vector is rotated into the sensor frame with the current attitude and subtracted from the angular rate
4. **Gyro offset tracking:** FusionOffset, seeded with the alignment result
5. **AHRS update:** Fusion (acceleration rejection 10°, gain scheduled in 7b)
6. **Gravity removal and bias correction** in the sensor frame, then rotation into the Earth frame
7. **ZUPT:** stationary when, over a 16-sample window, the variance is below the thresholds and the mean acceleration is below 0.1 m/s² and the mean angular rate below 2 °/s, for 96 consecutive samples (50 ms). While the estimated speed exceeds 0.1 m/s, 1 s of continuous stillness is required instead (a short constant-velocity phase in the middle of a move must not be taken for a stop). Leaving is immediate
7b. **Gain scheduling:** confirmed stationary → 0.5; moving (high variance, or large mean acceleration/rotation) → 0.02; low variance and low mean but not yet confirmed → 0.1. A "low variance but large mean acceleration" state lasting more than 2 s is assumed to be an attitude error at rest and falls back to 0.1
8. **Bias learning:** while stationary, the residual acceleration is learned with a 1 s time constant
9. **Integration:** trapezoidal velocity and position; velocity is reset to zero while stationary
10. **Retroactive ZUPT correction:** when a ZUPT is confirmed at the end of a motion segment of duration T, the remaining velocity v_err is error, and v_err × T / 2 is subtracted from the position. Segments in which the velocity limit was hit, or longer than 60 s, are not corrected

### Current performance on synthetic data (`metric` lines of `cd test && make`)

| Scenario | Position error |
|---|---|
| 0.3 m in 0.5 s (abrupt) | 0.6 mm |
| 1 m in 2 s (smooth) | 3.5 mm |
| 2 m in 3 s | 8.3 mm |
| 1 m in 4 s (slow, peak 0.36 m/s²) | 2.0 cm |
| 90° turn in place | 1.5 cm, attitude error 0.03° |

### Known limitations (next improvement targets)

- **Constant-velocity motion longer than 1 s is indistinguishable from rest.** This is inherent to a stand-alone IMU; shorter coasting phases are handled by the speed-dependent confirmation time
- **Slow motion** with accelerations around 0.1 m/s² sits at the mean-acceleration threshold and comes out short
- **Gyro-compass accuracy depends on the gyro bias.** If the bias exceeds the horizontal earth rate (about 0.0034 °/s in Tokyo) the initial heading is arbitrary (relative positioning is unaffected; the tests evaluate position error in the initial-heading-aligned frame)
- **The retroactive ZUPT correction produces a position step.** It is applied at the moment the ZUPT is confirmed, so the output position jumps; if continuity matters it should be spread over the following segment
- **Thresholds have not been tuned on real motion data.** The synthetic data includes noise, bias and earth rotation but not the real vibration signature

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
| `IMU_RING_SIZE` (shared_types.h) | 256 | inter-core ring buffer length |
| `IMU_NOTIFY_INTERVAL` (shared_types.h) | 8 | samples between wake-up notifications to SubCore1 (the receive FIFO has only 8 buffers) |

## Visualization

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
   - Check that the Spresense board package is installed
   - Check that the correct core is selected in the Tools menu

## Contributing

Contributions are welcome. Fork the repository and open a pull request; for larger changes please open an issue first. Run `cd test && make` before submitting.

## License

MIT License, see `LICENSE`.

---

For questions or problems open a GitHub issue or contact the repository maintainer.
