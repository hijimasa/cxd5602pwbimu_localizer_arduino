# Spresense IMU Self-Localization

English | [日本語](./README_jp.md)

This repository contains an Arduino IDE project for the Spresense board that uses the onboard IMU (Inertial Measurement Unit) to perform sensor-based self-localization. The project demonstrates sensor fusion techniques using the **Fusion library**, including calibration, filtering, and orientation estimation, to estimate acceleration, velocity, and position.

## Overview

The code reads raw IMU data (acceleration, rotation speed, and temperature) from the Spresense sensor and processes it through several steps:

- **Calibration & Initialization:** The IMU sensor data is averaged over a predefined measurement period to determine the initial bias and orientation.
- **Filtering:** A causal Gaussian filter is applied to smooth the sensor readings.
- **Orientation Estimation:** The program uses the **Fusion AHRS (Attitude and Heading Reference System)** library to fuse accelerometer and gyroscope data, providing robust quaternion-based orientation estimation with intelligent accelerometer rejection during motion.
- **Gyroscope Bias Compensation:** Runtime gyroscope offset correction is performed using the Fusion library's FusionOffset algorithm.
- **Acceleration Bias Correction:** An adaptive bias correction mechanism continuously updates acceleration bias during stationary periods.
- **Zero Velocity Update (ZUPT):** A zero-velocity update is implemented to reset drift when the sensor readings indicate near-zero movement, based on acceleration and gyroscope magnitude thresholds.
- **Position Estimation:** Velocity and position are updated using trapezoidal integration based on the corrected acceleration values.

## Features

- **IMU Data Acquisition:** Reads acceleration, gyroscope, and temperature data from the Spresense IMU.
- **Fusion AHRS Integration:** Utilizes the Fusion library for robust sensor fusion with accelerometer rejection and automatic gravity removal.
- **Quaternion-Based Orientation:** Provides smooth orientation updates using the Fusion AHRS algorithm.
- **Gaussian Filtering:** Uses a causal Gaussian filter to reduce high-frequency noise in sensor data.
- **Adaptive Bias Correction:** Continuously updates acceleration bias during stationary periods to compensate for temperature drift.
- **Zero-Velocity Update:** Applies ZUPT correction when the device is detected to be stationary, preventing velocity drift accumulation.
- **Trapezoidal Integration:** Uses trapezoidal method for velocity and position updates, improving accuracy over simple Euler integration.
- **Real-Time Data Output:** Logs key computed values (timestamp, sensor data, estimated acceleration, orientation, velocity, and position) via the Serial interface.

## Hardware Requirements

- **Sony Spresense Board:** Ensure your board includes the onboard IMU.
- **IMU Sensor:** The project uses the built-in CXD5602PWBIMU sensor.

## Software Requirements

- **Arduino IDE:** Compatible with the Spresense Arduino libraries.
- **Spresense SDK:** Required to compile and flash the firmware onto the board.

## Getting Started

### Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino.git
   cd cxd5602pwbimu_localizer_arduino
   git submodule update --init
   ```

2. **Open in Arduino IDE:**

   Open the project in the Arduino IDE. Make sure the Spresense board and related libraries are installed.

3. **Compile & Upload:**

   Build and upload the code to your Spresense board. The program automatically starts reading from the IMU once the board is running.

### Running the Application

- Once uploaded, open the Serial Monitor (set at 115200 baud) to view the output.
- The code prints formatted sensor data at regular intervals, including the current orientation (as a quaternion), velocity, and position estimates.

## Code Structure and Algorithm Details

### Initialization and Calibration

- **Initialization Function (`imu_data_initialize`):**
  Averages a number of samples (defined by `MESUREMENT_FREQUENCY`) to calibrate the accelerometer and gyroscope readings. It also computes the initial quaternion based on gravity and earth rotation.

- **Acceleration Bias Learning (`initialize_acceleration_bias`):**
  During the setup phase, the system collects 5 seconds of IMU data (9600 samples) to learn the initial acceleration bias using the Fusion AHRS library. This helps compensate for sensor manufacturing variations and temperature-dependent offsets.

### Data Processing

- **Gaussian Filtering:**
  A causal Gaussian filter smooths both the acceleration and rotation speed data using a precomputed kernel with sigma = LIST_SIZE/8.0.

- **Fusion AHRS Algorithm:**
  The Fusion library provides a comprehensive AHRS solution that handles:
  - Quaternion update with gyroscope integration
  - Accelerometer correction with intelligent rejection during motion (threshold: 10 degrees)
  - Automatic gravity vector tracking and removal
  - Coordinate frame transformations (North-West-Up convention)
  - Recovery mechanism after sensor saturation (5-second trigger period)

- **Gyroscope Offset Correction:**
  The `FusionOffset` algorithm continuously estimates and corrects gyroscope bias during runtime.

- **Adaptive Acceleration Bias Update:**
  During stationary periods (detected by ZUPT), the acceleration bias is continuously updated using exponential moving average (learning rate: 0.1%) to compensate for temperature drift.

- **Zero-Velocity Update (ZUPT):**
  The improved ZUPT algorithm detects stationary state based on:
  - Acceleration magnitude threshold: < 0.15 m/s²
  - Gyroscope magnitude threshold: < 0.015 rad/s (≈ 0.86°/s)
  - Duration requirement: 0.1 seconds (192 samples)

  When stationary state is detected, velocity is reset to zero and acceleration bias is updated.

### Integration for Velocity and Position

- **Velocity & Position Updates:**
  After compensating for gravity and bias, the acceleration is integrated over time using the **trapezoidal method** to update the velocity and then the position. This provides better accuracy compared to simple Euler integration.

## Contributing

Contributions are welcome! Please fork the repository and submit your pull requests. For major changes, please open an issue first to discuss what you would like to change.

## License

Distributed under the MIT License. See `LICENSE` for more information.

---

This README provides an overview of the project and explains the main processing steps implemented in the code. For further details or questions, please feel free to open an issue or contact the repository maintainer.
