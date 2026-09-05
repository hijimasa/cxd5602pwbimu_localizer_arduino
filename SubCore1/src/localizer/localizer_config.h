/**
 * @file localizer_config.h
 * @brief Tunable constants for the IMU localizer algorithm.
 *
 * This header has no Arduino dependency so that the algorithm can be
 * compiled and unit-tested on a host PC.
 */
#ifndef LOCALIZER_CONFIG_H
#define LOCALIZER_CONFIG_H

// ---- Physical constants ----------------------------------------------------
#define LOCALIZER_PI 3.14159265358979f
#define LOCALIZER_GRAVITY 9.7975f            // m/s^2, Tokyo Japan
#define LOCALIZER_LATITUDE_DEG 35.68f        // deg, Tokyo Japan (earth-rate compensation)
#define LOCALIZER_EARTH_RATE 7.2921159e-5f   // rad/s

// ---- Sensor / timing -------------------------------------------------------
#define LOCALIZER_SAMPLE_RATE 1920           // Hz, full CXD5602PWBIMU performance
#define LOCALIZER_TIMESTAMP_HZ 19200000.0f   // IMU timestamp counter frequency
#define LOCALIZER_NOMINAL_DT (1.0f / (float)LOCALIZER_SAMPLE_RATE)
// dt larger than this is treated as an anomaly (lost samples); it is still
// integrated but clamped to LOCALIZER_MAX_DT.
#define LOCALIZER_DT_ANOMALY_FACTOR 1.5f
#define LOCALIZER_MAX_DT (10.0f * LOCALIZER_NOMINAL_DT)

// ---- Initial alignment (sensor must be stationary) -------------------------
#define LOCALIZER_CALIBRATION_SECONDS 15
#define LOCALIZER_CALIBRATION_SAMPLES (LOCALIZER_CALIBRATION_SECONDS * LOCALIZER_SAMPLE_RATE)

// ---- Sensor noise model (from CXD5602PWBIMU datasheet) --------------------
#define GYRO_NOISE_DENSITY_RAD (1.0e-3f * LOCALIZER_PI / 180.0f)   // rad/s/sqrt(Hz)
#define ACCEL_NOISE_DENSITY (14.0e-6f * LOCALIZER_GRAVITY)         // m/s^2/sqrt(Hz)

// ---- Causal Gaussian pre-filter --------------------------------------------
#define GAUSSIAN_LIST_SIZE 16
#define GAUSSIAN_SIGMA 2.0f

// ---- Fusion AHRS -----------------------------------------------------------
#define FUSION_AHRS_GAIN 0.1f
#define FUSION_GYRO_RANGE 2000.0f
#define FUSION_ACCEL_REJECTION 10.0f
#define FUSION_RECOVERY_TRIGGER_PERIOD (5 * LOCALIZER_SAMPLE_RATE)

// ---- ZUPT (zero velocity update) detection ---------------------------------
#define ZUPT_ACCEL_VARIANCE_THRESHOLD 0.004f     // (m/s^2)^2, sum over 3 axes
#define ZUPT_GYRO_VARIANCE_THRESHOLD 0.00006f    // (rad/s)^2, sum over 3 axes
#define ZUPT_WINDOW_SIZE 16                      // samples used for variance
#define ZUPT_REQUIRED_SAMPLES (LOCALIZER_SAMPLE_RATE / 20) // 0.05 s confirmation

// ---- Accelerometer bias learning during ZUPT -------------------------------
#define BIAS_LEARNING_RATE (1.0f / (float)LOCALIZER_SAMPLE_RATE)

// ---- Velocity / position integration ---------------------------------------
#define LOCALIZER_MAX_VELOCITY 5.0f              // m/s, runaway guard

#endif // LOCALIZER_CONFIG_H
