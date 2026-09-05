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
// The gain is scheduled from the ZUPT detector state:
//   stationary (confirmed ZUPT)      -> FUSION_GAIN_STATIONARY: converge fast
//   still (low variance & low mean)
//     but not yet confirmed          -> FUSION_GAIN_SETTLING
//   moving (vibration or sustained
//     acceleration/rotation)         -> FUSION_GAIN_MOTION: trust the gyro,
//                                       linear acceleration must not tilt us
// A sustained "low variance but large mean acceleration" state is either a
// smooth move or an attitude error at rest. It is treated as motion first;
// if it persists longer than FUSION_AMBIGUOUS_TIMEOUT it is assumed to be an
// attitude error and the settling gain is used so that it converges.
#define FUSION_AHRS_GAIN 0.1f            // initial / settling gain
#define FUSION_GAIN_STATIONARY 0.5f
#define FUSION_GAIN_SETTLING FUSION_AHRS_GAIN
#define FUSION_GAIN_MOTION 0.02f
#define FUSION_AMBIGUOUS_TIMEOUT (2 * LOCALIZER_SAMPLE_RATE) // samples (2 s)
#define FUSION_GYRO_RANGE 2000.0f
#define FUSION_ACCEL_REJECTION 10.0f
#define FUSION_RECOVERY_TRIGGER_PERIOD (5 * LOCALIZER_SAMPLE_RATE)

// ---- ZUPT (zero velocity update) detection ---------------------------------
// Two conditions over a short window, both must hold for ZUPT_REQUIRED_SAMPLES:
//  * variance (sum over axes) below threshold: no vibration / jerk
//  * mean magnitude below threshold: no sustained acceleration / rotation.
//    Without it a locally constant acceleration (extremum of a smooth move)
//    is indistinguishable from rest.
#define ZUPT_ACCEL_VARIANCE_THRESHOLD 0.004f     // (m/s^2)^2, sum over 3 axes
#define ZUPT_GYRO_VARIANCE_THRESHOLD 0.00006f    // (rad/s)^2, sum over 3 axes
#define ZUPT_ACCEL_MEAN_THRESHOLD 0.1f           // m/s^2, |mean earth accel|
#define ZUPT_GYRO_MEAN_THRESHOLD (2.0f * LOCALIZER_PI / 180.0f) // rad/s, |mean rate|
#define ZUPT_WINDOW_SIZE 16                      // samples used for the statistics
#define ZUPT_REQUIRED_SAMPLES (LOCALIZER_SAMPLE_RATE / 20) // 0.05 s confirmation
// A brief zero-acceleration phase in the middle of a smooth move (constant
// velocity) looks exactly like rest. A genuine stop ends with the estimated
// speed near zero, so when the speed is still large the "still" condition
// must persist much longer before the velocity is reset.
#define ZUPT_SLOW_CONFIRM_SPEED 0.1f             // m/s
#define ZUPT_SLOW_CONFIRM_SAMPLES (LOCALIZER_SAMPLE_RATE * 1) // 1.0 s

// ---- Accelerometer bias learning during ZUPT -------------------------------
#define BIAS_LEARNING_RATE (1.0f / (float)LOCALIZER_SAMPLE_RATE)

// ---- Velocity / position integration ---------------------------------------
#define LOCALIZER_MAX_VELOCITY 5.0f              // m/s, runaway guard

#endif // LOCALIZER_CONFIG_H
