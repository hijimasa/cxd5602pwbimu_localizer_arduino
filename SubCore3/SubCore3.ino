/**
 * @file SubCore3.ino
 * @brief ZUPT (Zero Velocity Update) and Bias Processing Core
 *
 * SubCore3: Receives AHRS data, performs ZUPT detection and
 * continuous bias update, then forwards to SubCore4 (Position)
 */

#if (SUBCORE != 3)
#error "Core selection is wrong!! Must select SubCore3"
#endif

#include <MP.h>
#include "../shared_types.h"

// ZUPT detection parameters
#define ACCEL_THRESHOLD 0.05f    // m/s^2
#define GYRO_THRESHOLD 0.005f    // rad/s (~0.86 deg/s)
#define REQUIRED_SAMPLES (MESUREMENT_FREQUENCY / 10)  // 0.1 second

// Bias learning parameters
#define BIAS_LEARNING_RATE 0.01f

// State variables
static int zero_velocity_counter = 0;
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};
static bool bias_initialized = false;
static int bias_update_counter = 0;

bool detectStationary(float accel_magnitude, float gyro_magnitude)
{
  if (accel_magnitude < ACCEL_THRESHOLD && gyro_magnitude < GYRO_THRESHOLD) {
    zero_velocity_counter++;
  } else {
    zero_velocity_counter = 0;
  }

  return (zero_velocity_counter > REQUIRED_SAMPLES);
}

void updateBias(float sensor_ax, float sensor_ay, float sensor_az)
{
  accel_bias[0] += BIAS_LEARNING_RATE * sensor_ax;
  accel_bias[1] += BIAS_LEARNING_RATE * sensor_ay;
  accel_bias[2] += BIAS_LEARNING_RATE * sensor_az;

  // Periodically send updated bias to SubCore2
  bias_update_counter++;
  if (bias_update_counter >= MESUREMENT_FREQUENCY) {  // Every 1 second
    static BiasData_t bias_data;
    bias_data.accel_bias[0] = accel_bias[0];
    bias_data.accel_bias[1] = accel_bias[1];
    bias_data.accel_bias[2] = accel_bias[2];
    bias_data.initialized = bias_initialized;
    bias_data.sample_count = 0;

    MP.Send(MSG_ID_BIAS_DATA, &bias_data, SUBCORE_AHRS);
    bias_update_counter = 0;
  }
}

void setup()
{
  // Initialize MP library
  MP.begin();

  // Set receive timeout to blocking mode
  MP.RecvTimeout(MP_RECV_BLOCKING);
}

void loop()
{
  int8_t msgid;
  void *msgdata;

  // Wait for message
  int ret = MP.Recv(&msgid, &msgdata);
  if (ret < 0) {
    return;
  }

  // Handle initialization message
  if (msgid == MSG_ID_INIT_COMPLETE) {
    InitParams_t *init_params = (InitParams_t *)msgdata;
    accel_bias[0] = init_params->initial_accel_bias[0];
    accel_bias[1] = init_params->initial_accel_bias[1];
    accel_bias[2] = init_params->initial_accel_bias[2];
    bias_initialized = init_params->bias_initialized;
    return;
  }

  // Process AHRS data
  if (msgid != MSG_ID_AHRS) {
    return;
  }

  AhrsData_t *ahrs = (AhrsData_t *)msgdata;

  // Calculate magnitudes for ZUPT detection
  float accel_magnitude = sqrt(
    ahrs->earth_accel[0] * ahrs->earth_accel[0] +
    ahrs->earth_accel[1] * ahrs->earth_accel[1] +
    ahrs->earth_accel[2] * ahrs->earth_accel[2]
  );

  float gyro_magnitude = sqrt(
    ahrs->gyro_corrected[0] * ahrs->gyro_corrected[0] +
    ahrs->gyro_corrected[1] * ahrs->gyro_corrected[1] +
    ahrs->gyro_corrected[2] * ahrs->gyro_corrected[2]
  );

  // ZUPT detection
  bool is_stationary = detectStationary(accel_magnitude, gyro_magnitude);

  // Update bias during stationary periods
  if (is_stationary && bias_initialized) {
    updateBias(ahrs->sensor_accel[0], ahrs->sensor_accel[1], ahrs->sensor_accel[2]);
  }

  // Prepare output data
  static ZuptData_t zupt_data;
  zupt_data.timestamp = ahrs->timestamp;
  zupt_data.quaternion[0] = ahrs->quaternion[0];
  zupt_data.quaternion[1] = ahrs->quaternion[1];
  zupt_data.quaternion[2] = ahrs->quaternion[2];
  zupt_data.quaternion[3] = ahrs->quaternion[3];
  zupt_data.earth_accel[0] = ahrs->earth_accel[0];
  zupt_data.earth_accel[1] = ahrs->earth_accel[1];
  zupt_data.earth_accel[2] = ahrs->earth_accel[2];
  zupt_data.gyro_magnitude = gyro_magnitude;
  zupt_data.accel_magnitude = accel_magnitude;
  zupt_data.is_stationary = is_stationary;
  zupt_data.dt = ahrs->dt;

  // Send to SubCore4 (Position)
  MP.Send(MSG_ID_ZUPT, &zupt_data, SUBCORE_POSITION);
}
