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
- **Variance based ZUPT:** stationary detection from signal variance; velocity is reset while stationary
- **Accelerometer bias learning** while stationary
- **Trapezoidal integration** of velocity and position
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
5. **AHRS update:** Fusion (gain 0.1, acceleration rejection 10°)
6. **Gravity removal and bias correction** in the sensor frame, then rotation into the Earth frame
7. **ZUPT:** stationary when the variance over a 16-sample window stays below the thresholds for 96 consecutive samples (50 ms); leaving is immediate
8. **Bias learning:** while stationary, the residual acceleration is learned with a 1 s time constant
9. **Integration:** trapezoidal velocity and position; velocity is reset to zero while stationary

### Known issues (next improvement targets)

- **The variance-only ZUPT misclassifies smooth motion as stationary.** Wherever the acceleration is locally constant (constant velocity, acceleration extrema) the variance drops below the threshold and the velocity is zeroed in the middle of a move. The `known_issue` tests in `test/test_localizer.cpp` record the current behaviour
- **The AHRS gain is fixed during motion**, so translational acceleration pulls the attitude estimate
- **Gyro-compass accuracy depends on the gyro bias.** If the bias exceeds the horizontal earth rate (about 0.0034 °/s in Tokyo) the initial heading is arbitrary (relative positioning is unaffected)

## Configuration

Main parameters in `SubCore1/src/localizer/localizer_config.h`:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `LOCALIZER_SAMPLE_RATE` | 1920 Hz | IMU sample rate |
| `LOCALIZER_GRAVITY` | 9.7975 m/s² | local gravity (Tokyo) |
| `LOCALIZER_LATITUDE_DEG` | 35.68° | latitude for earth-rate compensation (Tokyo) |
| `LOCALIZER_CALIBRATION_SECONDS` | 15 s | stationary calibration period at start-up |
| `FUSION_AHRS_GAIN` | 0.1 | AHRS filter gain |
| `FUSION_ACCEL_REJECTION` | 10.0° | acceleration rejection threshold |
| `ZUPT_ACCEL_VARIANCE_THRESHOLD` | 0.004 (m/s²)² | ZUPT acceleration variance threshold |
| `ZUPT_GYRO_VARIANCE_THRESHOLD` | 0.00006 (rad/s)² | ZUPT angular-rate variance threshold |
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
