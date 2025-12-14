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
#include "shared_types.h"

// ZUPT detection parameters - tuned for reliable stationary detection
// Key insight: use VARIANCE (not absolute value) to detect stationary state
// A stationary sensor has low variance even with bias offset
#define ACCEL_VARIANCE_THRESHOLD 0.004f              // (m/s^2)^2 - variance threshold for accel
#define GYRO_VARIANCE_THRESHOLD 0.00006f             // (rad/s)^2 - variance threshold for gyro
#define REQUIRED_SAMPLES (MESUREMENT_FREQUENCY / 20) // 0.05 second = 96 samples at 1920Hz
#define MOTION_WINDOW_SIZE 16                        // Window for variance calculation

// Bias learning parameters
// Old implementation used alpha = 0.01 at 240Hz
// Scale to maintain equivalent convergence at higher sample rate
#define BIAS_LEARNING_RATE (0.001f)

// State variables
static int zero_velocity_counter = 0;
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};
static bool bias_initialized = false;
static bool init_complete = false; // Wait for initialization from MainCore

// Buffers for variance calculation (store raw values, not magnitudes)
static float accel_x_buffer[MOTION_WINDOW_SIZE];
static float accel_y_buffer[MOTION_WINDOW_SIZE];
static float accel_z_buffer[MOTION_WINDOW_SIZE];
static float gyro_x_buffer[MOTION_WINDOW_SIZE];
static float gyro_y_buffer[MOTION_WINDOW_SIZE];
static float gyro_z_buffer[MOTION_WINDOW_SIZE];
static int motion_buffer_idx = 0;
static bool buffer_filled = false;

// Statistics tracking
static uint32_t loop_count = 0;
static uint32_t last_stats_time = 0;
static uint32_t stats_loop_count_start = 0;
static uint32_t recv_error_count = 0;
static uint32_t wrong_msgid_count = 0;

// Calculate variance of a buffer
static float calculateVariance(const float *buffer, int size)
{
  // Calculate mean
  float mean = 0.0f;
  for (int i = 0; i < size; i++)
  {
    mean += buffer[i];
  }
  mean /= size;

  // Calculate variance
  float variance = 0.0f;
  for (int i = 0; i < size; i++)
  {
    float diff = buffer[i] - mean;
    variance += diff * diff;
  }
  return variance / size;
}

bool detectStationary(float ax, float ay, float az, float gx, float gy, float gz)
{
  // Update buffers with raw values
  accel_x_buffer[motion_buffer_idx] = ax;
  accel_y_buffer[motion_buffer_idx] = ay;
  accel_z_buffer[motion_buffer_idx] = az;
  gyro_x_buffer[motion_buffer_idx] = gx;
  gyro_y_buffer[motion_buffer_idx] = gy;
  gyro_z_buffer[motion_buffer_idx] = gz;

  motion_buffer_idx = (motion_buffer_idx + 1) % MOTION_WINDOW_SIZE;
  if (motion_buffer_idx == 0)
  {
    buffer_filled = true;
  }

  // Need full buffer for accurate variance
  if (!buffer_filled)
  {
    return false;
  }

  // Calculate variance for each axis
  float accel_var_x = calculateVariance(accel_x_buffer, MOTION_WINDOW_SIZE);
  float accel_var_y = calculateVariance(accel_y_buffer, MOTION_WINDOW_SIZE);
  float accel_var_z = calculateVariance(accel_z_buffer, MOTION_WINDOW_SIZE);
  float gyro_var_x = calculateVariance(gyro_x_buffer, MOTION_WINDOW_SIZE);
  float gyro_var_y = calculateVariance(gyro_y_buffer, MOTION_WINDOW_SIZE);
  float gyro_var_z = calculateVariance(gyro_z_buffer, MOTION_WINDOW_SIZE);

  // Total variance (sum of all axes)
  float total_accel_var = accel_var_x + accel_var_y + accel_var_z;
  float total_gyro_var = gyro_var_x + gyro_var_y + gyro_var_z;

  // Check if variance is below threshold (stationary = low variance)
  if (total_accel_var < ACCEL_VARIANCE_THRESHOLD && total_gyro_var < GYRO_VARIANCE_THRESHOLD)
  {
    zero_velocity_counter++;
  }
  else
  {
    zero_velocity_counter = 0;
  }

  return (zero_velocity_counter > REQUIRED_SAMPLES);
}

void updateBias(float sensor_ax, float sensor_ay, float sensor_az)
{
  // During stationary period, sensor_accel should be zero
  // Any non-zero value is bias error, so we update to reduce it
  // Use small learning rate to slowly converge
  accel_bias[0] += BIAS_LEARNING_RATE * sensor_ax;
  accel_bias[1] += BIAS_LEARNING_RATE * sensor_ay;
  accel_bias[2] += BIAS_LEARNING_RATE * sensor_az;

  {
    static BiasData_t bias_data;
    bias_data.accel_bias[0] = accel_bias[0];
    bias_data.accel_bias[1] = accel_bias[1];
    bias_data.accel_bias[2] = accel_bias[2];
    bias_data.initialized = bias_initialized;
    bias_data.sample_count = 0;

    MP.Send(MSG_ID_BIAS_DATA, &bias_data, 0); // 0 = MainCore (will forward)
  }
}

void setup()
{
  // Initialize MP library
  MP.begin();

  // Initialize variance calculation buffers
  for (int i = 0; i < MOTION_WINDOW_SIZE; i++)
  {
    accel_x_buffer[i] = 0.0f;
    accel_y_buffer[i] = 0.0f;
    accel_z_buffer[i] = 0.0f;
    gyro_x_buffer[i] = 0.0f;
    gyro_y_buffer[i] = 0.0f;
    gyro_z_buffer[i] = 0.0f;
  }

  // Set receive timeout to blocking mode
  MP.RecvTimeout(MP_RECV_BLOCKING);

  last_stats_time = millis();

  // Send startup notification to MainCore
  static ProcessingStats_t startup_stats;
  startup_stats.core_id = 3;
  startup_stats.loop_count = 0;
  startup_stats.actual_rate_hz = 0.0f;
  startup_stats.dropped_messages = 0;
  MP.Send(MSG_ID_STATS, &startup_stats, 0);
}

void loop()
{
  int8_t msgid;
  void *msgdata;

  // Wait for message
  int ret = MP.Recv(&msgid, &msgdata);
  if (ret < 0)
  {
    recv_error_count++;
    return;
  }

  // Handle initialization message
  if (msgid == MSG_ID_INIT_COMPLETE)
  {
    InitParams_t *init_params = (InitParams_t *)msgdata;
    accel_bias[0] = init_params->initial_accel_bias[0];
    accel_bias[1] = init_params->initial_accel_bias[1];
    accel_bias[2] = init_params->initial_accel_bias[2];
    bias_initialized = init_params->bias_initialized;
    init_complete = true; // Now ready to process data
    return;
  }

  // Skip data processing until initialization is complete
  if (!init_complete)
  {
    return;
  }

  // Process AHRS data
  if (msgid != MSG_ID_AHRS)
  {
    wrong_msgid_count++;
    return;
  }

  AhrsData_t *ahrs = (AhrsData_t *)msgdata;

  // Calculate magnitudes for output (still needed for SubCore4 decay calculation)
  float accel_magnitude = sqrt(
      ahrs->earth_accel[0] * ahrs->earth_accel[0] +
      ahrs->earth_accel[1] * ahrs->earth_accel[1] +
      ahrs->earth_accel[2] * ahrs->earth_accel[2]);

  float gyro_magnitude = sqrt(
      ahrs->gyro_filtered[0] * ahrs->gyro_filtered[0] +
      ahrs->gyro_filtered[1] * ahrs->gyro_filtered[1] +
      ahrs->gyro_filtered[2] * ahrs->gyro_filtered[2]);

  // ZUPT detection using variance-based method
  // Pass raw values for variance calculation
  bool is_stationary = detectStationary(
      ahrs->earth_accel[0], ahrs->earth_accel[1], ahrs->earth_accel[2],
      ahrs->gyro_filtered[0], ahrs->gyro_filtered[1], ahrs->gyro_filtered[2]);

  if (is_stationary)
  {
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
  zupt_data.sensor_accel[0] = ahrs->sensor_accel[0];
  zupt_data.sensor_accel[1] = ahrs->sensor_accel[1];
  zupt_data.sensor_accel[2] = ahrs->sensor_accel[2];
  zupt_data.linear_accel_raw[0] = ahrs->linear_accel_raw[0];
  zupt_data.linear_accel_raw[1] = ahrs->linear_accel_raw[1];
  zupt_data.linear_accel_raw[2] = ahrs->linear_accel_raw[2];
  zupt_data.gyro_corrected[0] = ahrs->gyro_corrected[0];
  zupt_data.gyro_corrected[1] = ahrs->gyro_corrected[1];
  zupt_data.gyro_corrected[2] = ahrs->gyro_corrected[2];
  zupt_data.gyro_filtered[0] = ahrs->gyro_filtered[0];
  zupt_data.gyro_filtered[1] = ahrs->gyro_filtered[1];
  zupt_data.gyro_filtered[2] = ahrs->gyro_filtered[2];
  zupt_data.gyro_magnitude = gyro_magnitude;
  zupt_data.accel_magnitude = accel_magnitude;
  zupt_data.is_stationary = is_stationary;
  zupt_data.dt = ahrs->dt;
  zupt_data.temp = ahrs->temp;

  // Send to MainCore (which will forward to SubCore4)
  MP.Send(MSG_ID_ZUPT, &zupt_data, 0); // 0 = MainCore

  // Update loop count
  loop_count++;

  // Send statistics every 5 seconds
  uint32_t current_time = millis();
  if (current_time - last_stats_time >= 5000)
  {
    static ProcessingStats_t stats;
    stats.core_id = 3;
    stats.loop_count = loop_count;
    stats.actual_rate_hz = (loop_count - stats_loop_count_start) / 5.0f;
    stats.dropped_messages = recv_error_count + wrong_msgid_count;

    MP.Send(MSG_ID_STATS, &stats, 0); // Send to MainCore

    last_stats_time = current_time;
    stats_loop_count_start = loop_count;

    // Reset error counters after reporting
    recv_error_count = 0;
    wrong_msgid_count = 0;
  }
}
