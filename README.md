# Spresense IMU Self-Localization

English | [日本語](./README_jp.md)

This repository contains an Arduino IDE project for the Spresense board that uses the onboard IMU (Inertial Measurement Unit) to perform sensor-based self-localization. The project leverages the **Spresense multicore architecture** (6 cores) for high-performance parallel processing, implementing sensor fusion techniques using the **Fusion library** for calibration, filtering, and orientation estimation.

## Overview

The system reads raw IMU data (acceleration, rotation speed, and temperature) at 1920Hz and processes it through a multicore pipeline:

- **MainCore:** IMU data acquisition, initial calibration, and data distribution
- **SubCore1:** Gaussian filtering for noise reduction
- **SubCore2:** AHRS (Attitude and Heading Reference System) using Fusion library
- **SubCore3:** Zero Velocity Update (ZUPT) detection and bias estimation
- **SubCore4:** Velocity and position integration

## Architecture

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│  MainCore   │───▶│  SubCore1   │───▶│  SubCore2   │───▶│  SubCore3   │───▶│  SubCore4   │
│  IMU Read   │    │  Gaussian   │    │  Fusion     │    │    ZUPT     │    │  Position   │
│  1920Hz     │    │   Filter    │    │    AHRS     │    │    Bias     │    │ Integration │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘    └──────┬──────┘
                                                                                    │
                                                                                    ▼
                                                                           Serial Output (30Hz)
```

## Features

- **High-Speed IMU Data Acquisition:** 1920Hz sampling rate utilizing full IMU performance
- **Multicore Parallel Processing:** Distributes computation across 5 cores for real-time processing
- **Fusion AHRS Integration:** Robust sensor fusion with accelerometer rejection and automatic gravity removal
- **Gaussian Filtering:** Causal Gaussian filter for noise reduction
- **Variance-Based ZUPT Detection:** Detects stationary state using signal variance instead of absolute thresholds, enabling accurate detection even with sensor bias
- **Adaptive Bias Correction:** Continuously updates acceleration bias during stationary periods
- **Trapezoidal Integration:** Improved accuracy for velocity and position estimation

## Hardware Requirements

- **Sony Spresense Main Board** with CXD5602 processor
- **Spresense IMU Add-on Board** (CXD5602PWBIMU)

## Software Requirements

- **Arduino IDE** (1.8.x or 2.x)
- **Spresense Arduino Board Package** (3.0.0 or later)
- **Spresense SDK** (for board support)

## Getting Started

### Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino.git
   cd cxd5602pwbimu_localizer_arduino
   git submodule update --init
   ```

2. **Install Spresense Arduino Board Package:**

   - Open Arduino IDE
   - Go to **File** > **Preferences**
   - Add the following URL to "Additional Board Manager URLs":
     ```
     https://github.com/sonydevworld/spresense-arduino-compatible/releases/download/generic/package_spresense_index.json
     ```
   - Go to **Tools** > **Board** > **Board Manager**
   - Search for "Spresense" and install the package

### Building and Uploading (Multicore)

Since this project uses multiple cores, you need to compile and upload each core separately in the Arduino IDE.

#### Step 1: Upload SubCores First

You must upload SubCores **before** uploading the MainCore.

1. **SubCore1 (Gaussian Filter):**
   - Open `SubCore1/SubCore1.ino` in Arduino IDE
   - Go to **Tools** > **Core** > Select **"SubCore 1"**
   - Click **Upload** (or Ctrl+U)

2. **SubCore2 (AHRS):**
   - Open `SubCore2/SubCore2.ino` in Arduino IDE
   - Go to **Tools** > **Core** > Select **"SubCore 2"**
   - Click **Upload**

3. **SubCore3 (ZUPT):**
   - Open `SubCore3/SubCore3.ino` in Arduino IDE
   - Go to **Tools** > **Core** > Select **"SubCore 3"**
   - Click **Upload**

4. **SubCore4 (Position Integration):**
   - Open `SubCore4/SubCore4.ino` in Arduino IDE
   - Go to **Tools** > **Core** > Select **"SubCore 4"**
   - Click **Upload**

#### Step 2: Upload MainCore Last

1. Open `cxd5602pwbimu_localizer_arduino.ino` in Arduino IDE
2. Go to **Tools** > **Core** > Select **"MainCore"**
3. Click **Upload**

### Running the Application

1. After uploading all cores, open the Serial Monitor
2. Set baud rate to **115200**
3. The system will perform automatic calibration:
   - **5 seconds:** Initial orientation calibration (keep the sensor stationary)
   - **10 seconds:** Acceleration bias learning (keep the sensor stationary)
4. After calibration, the system outputs position data at 30Hz

### Output Format

The serial output contains comma-separated values:
```
timestamp, qw, qx, qy, qz, vx, vy, vz, px, py, pz, ax, ay, az, gx, gy, gz, temp, is_stationary
```

## Project Structure

```
cxd5602pwbimu_localizer_arduino/
├── cxd5602pwbimu_localizer_arduino.ino  # MainCore: IMU acquisition & calibration
├── shared_types.h                        # Shared data structures for inter-core communication
├── SubCore1/
│   └── SubCore1.ino                      # Gaussian filter processing
├── SubCore2/
│   ├── SubCore2.ino                      # Fusion AHRS processing
│   └── src/Fusion/                       # Fusion library
├── SubCore3/
│   └── SubCore3.ino                      # ZUPT detection & bias estimation
├── SubCore4/
│   └── SubCore4.ino                      # Velocity & position integration
├── SubCore5/
│   └── SubCore5.ino                      # (Reserved for future use)
└── src/Fusion/                           # Fusion library for MainCore
```

## Algorithm Details

### Initialization and Calibration

1. **Orientation Calibration (5 seconds):**
   - Averages accelerometer and gyroscope readings
   - Calculates initial quaternion from gravity and Earth rotation vectors

2. **Acceleration Bias Learning (10 seconds):**
   - Uses Fusion AHRS to compute gravity-removed acceleration
   - Learns sensor bias by comparing measured vs. expected gravity vector

### Data Processing Pipeline

1. **Gaussian Filter (SubCore1):**
   - Causal Gaussian filter with σ = LIST_SIZE/8.0
   - Reduces high-frequency noise while preserving signal dynamics

2. **AHRS Processing (SubCore2):**
   - Fusion library provides quaternion-based orientation estimation
   - Accelerometer rejection during dynamic motion (threshold: 10°)
   - Automatic gravity removal in Earth frame
   - Bias correction applied in sensor frame before coordinate transformation

3. **ZUPT Detection (SubCore3):**
   - **Variance-based detection:** Uses signal variance instead of absolute magnitude
   - Thresholds: Accel variance < 0.004 (m/s²)², Gyro variance < 0.00006 (rad/s)²
   - Window size: 16 samples with 96-sample (0.05s) confirmation period
   - Advantage: Works correctly even with sensor bias offset

4. **Position Integration (SubCore4):**
   - Trapezoidal integration for velocity and position
   - ZUPT correction resets velocity to zero during stationary periods

## Configuration

Key parameters in `shared_types.h`:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `MESUREMENT_FREQUENCY` | 1920 Hz | IMU sampling rate |
| `GRAVITY_AMOUNT` | 9.7975 m/s² | Local gravity (Tokyo, Japan) |
| `FUSION_AHRS_GAIN` | 0.1 | AHRS filter gain |
| `FUSION_ACCEL_REJECTION` | 10.0° | Accelerometer rejection threshold |

## Troubleshooting

### Common Issues

1. **SubCore fails to start:**
   - Ensure all SubCores are uploaded

2. **Position drift during stationary:**
   - Keep the sensor completely still during initial calibration
   - The variance-based ZUPT should handle residual bias

3. **Compilation errors:**
   - Ensure Spresense board package is properly installed
   - Check that the correct core is selected in Tools menu

## Contributing

Contributions are welcome! Please fork the repository and submit pull requests. For major changes, please open an issue first to discuss what you would like to change.

## License

Distributed under the MIT License. See `LICENSE` for more information.

---

For questions or issues, please open a GitHub issue or contact the repository maintainer.
