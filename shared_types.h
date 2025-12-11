/**
 * @file shared_types.h
 * @brief Shared data types for multicore IMU processing
 *
 * This header defines structures used for inter-core communication
 * on the Spresense 6-core system.
 */

#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// Configuration
#define GRAVITY_AMOUNT 9.7975f // Tokyo Japan
#define EARTH_ROTATION_SPEED_AMOUNT 7.2921159e-5
#define MESUREMENT_FREQUENCY 1920 // Full IMU performance
#define GYRO_NOISE_DENSITY (1.0e-3 * M_PI / 180.0f)
#define ACCEL_NOISE_DENSITY (14.0e-6 * GRAVITY_AMOUNT)
#define GYRO_NOISE_AMOUNT (GYRO_NOISE_DENSITY * sqrt(MESUREMENT_FREQUENCY))
#define ACCEL_NOISE_AMOUNT (ACCEL_NOISE_DENSITY * sqrt(MESUREMENT_FREQUENCY))
#define ACCEL_BIAS_DRIFT (4.43e-6 * GRAVITY_AMOUNT * 3.0f)
#define GYRO_BIAS_DRIFT (0.39f * M_PI / 180.0f)
#define GYRO_OBSERVATION_NOISE_VARIANCE (GYRO_NOISE_AMOUNT * GYRO_NOISE_AMOUNT)
#define ACCEL_OBSERVATION_NOISE_VARIANCE (ACCEL_NOISE_AMOUNT * ACCEL_NOISE_AMOUNT)
#define PROCESS_NOISE_VARIANCE (1.0e-7)

#define LIST_SIZE 4
#define SIGMA_K 1.0f

// Fusion AHRS settings
#define FUSION_AHRS_GAIN 0.5f
#define FUSION_GYRO_RANGE 2000.0f
#define FUSION_ACCEL_REJECTION 10.0f
#define FUSION_RECOVERY_TRIGGER_PERIOD (5 * MESUREMENT_FREQUENCY)

// Message IDs for inter-core communication
#define MSG_ID_IMU_RAW        0x01  // Raw IMU data from MainCore
#define MSG_ID_FILTERED       0x02  // Filtered data from SubCore1
#define MSG_ID_AHRS           0x03  // AHRS result from SubCore2
#define MSG_ID_ZUPT           0x04  // ZUPT/Bias result from SubCore3
#define MSG_ID_POSITION       0x05  // Position result from SubCore4
#define MSG_ID_OUTPUT         0x06  // Output request to SubCore5
#define MSG_ID_INIT_COMPLETE  0x10  // Initialization complete signal
#define MSG_ID_BIAS_DATA      0x11  // Bias calibration data

// SubCore IDs
#define SUBCORE_FILTER    1  // Gaussian filter
#define SUBCORE_AHRS      2  // Fusion AHRS
#define SUBCORE_ZUPT      3  // ZUPT & Bias
#define SUBCORE_POSITION  4  // Velocity & Position integration
#define SUBCORE_OUTPUT    5  // Serial output

/**
 * @brief Raw IMU data structure (from sensor)
 */
typedef struct {
  uint32_t timestamp;
  float temp;
  float ax;  // m/s^2
  float ay;
  float az;
  float gx;  // rad/s
  float gy;
  float gz;
} ImuRawData_t;

/**
 * @brief Filtered IMU data (after Gaussian filter)
 */
typedef struct {
  uint32_t timestamp;
  float ax;  // m/s^2 (filtered)
  float ay;
  float az;
  float gx;  // rad/s (filtered)
  float gy;
  float gz;
  float dt;  // time delta in seconds
} ImuFilteredData_t;

/**
 * @brief AHRS output data
 */
typedef struct {
  uint32_t timestamp;
  float quaternion[4];     // [w, x, y, z]
  float earth_accel[3];    // [x, y, z] m/s^2 (gravity removed, earth frame)
  float sensor_accel[3];   // [x, y, z] m/s^2 (gravity removed, sensor frame)
  float gyro_corrected[3]; // [x, y, z] rad/s (bias corrected)
  float dt;
} AhrsData_t;

/**
 * @brief ZUPT and bias processing result
 */
typedef struct {
  uint32_t timestamp;
  float quaternion[4];
  float earth_accel[3];
  float gyro_magnitude;
  float accel_magnitude;
  bool is_stationary;      // ZUPT判定結果
  float dt;
} ZuptData_t;

/**
 * @brief Position estimation result
 */
typedef struct {
  uint32_t timestamp;
  float quaternion[4];
  float velocity[3];
  float position[3];
  float acceleration[3];
  float gyro[3];
  float temp;
  bool is_stationary;
} PositionData_t;

/**
 * @brief Bias calibration data
 */
typedef struct {
  float accel_bias[3];
  bool initialized;
  int sample_count;
} BiasData_t;

/**
 * @brief Initialization parameters (MainCore -> SubCores)
 */
typedef struct {
  float initial_quaternion[4];
  float initial_accel_bias[3];
  bool bias_initialized;
} InitParams_t;

#endif // SHARED_TYPES_H
