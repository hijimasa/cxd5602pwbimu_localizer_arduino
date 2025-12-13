/**
 * @file SubCore1.ino
 * @brief Gaussian Filter Processing Core
 *
 * SubCore1: Receives raw IMU data and applies causal Gaussian filter
 * for noise reduction, then forwards to SubCore2 (AHRS)
 */

#if (SUBCORE != 1)
#error "Core selection is wrong!! Must select SubCore1"
#endif

#include <MP.h>
#include "shared_types.h"

// Ring buffer for Gaussian filter
static float measured_accel_x[LIST_SIZE];
static float measured_accel_y[LIST_SIZE];
static float measured_accel_z[LIST_SIZE];
static float measured_gyro_x[LIST_SIZE];
static float measured_gyro_y[LIST_SIZE];
static float measured_gyro_z[LIST_SIZE];
static int current_list_num = 0;
static int old_timestamp = -1;
static bool buffer_initialized = false;

// Gaussian kernel (initialized once)
static float kernel[LIST_SIZE];
static bool kernel_initialized = false;

// Statistics tracking
static uint32_t loop_count = 0;
static uint32_t last_stats_time = 0;
static uint32_t stats_loop_count_start = 0;
static uint32_t recv_error_count = 0;
static uint32_t wrong_msgid_count = 0;
static uint32_t send_error_count = 0;

void initializeKernel()
{
  float sum = 0.0f;
  for (int i = 0; i < LIST_SIZE; i++)
  {
    kernel[i] = (1.0f / (sqrt(2.0f * M_PI) * SIGMA_K)) *
                exp(-(float)(i * i) / (2.0f * SIGMA_K * SIGMA_K));
    sum += kernel[i];
  }
  // Normalize
  for (int i = 0; i < LIST_SIZE; i++)
  {
    kernel[i] /= sum;
  }
  kernel_initialized = true;
}

float applyCausalGaussianFilter(const float *x, int list_num)
{
  float y_current = 0.0f;
  for (int i = 0; i < LIST_SIZE; i++)
  {
    int idx = LIST_SIZE - 1 - i;
    int list_idx = (list_num + idx) % LIST_SIZE;
    y_current += kernel[i] * x[list_idx];
  }
  return y_current;
}

void setup()
{
  // Initialize MP library
  MP.begin();

  // Initialize Gaussian kernel
  initializeKernel();

  // Ring buffers will be initialized with first real data
  // to avoid transient response from zero-filled buffers

  // Set receive timeout to blocking mode for real-time processing
  MP.RecvTimeout(MP_RECV_BLOCKING);

  last_stats_time = millis();

  // Send startup notification to MainCore
  static ProcessingStats_t startup_stats;
  startup_stats.core_id = 1;
  startup_stats.loop_count = 0;
  startup_stats.actual_rate_hz = 0.0f;
  startup_stats.dropped_messages = 0;
  MP.Send(MSG_ID_STATS, &startup_stats, 0);
}

void loop()
{
  int8_t msgid;
  ImuRawData_t *raw_data;

  // Wait for raw IMU data from MainCore
  int ret = MP.Recv(&msgid, &raw_data);
  if (ret < 0)
  {
    recv_error_count++;
    return;
  }

  if (msgid != MSG_ID_IMU_RAW)
  {
    wrong_msgid_count++;
    return;
  }

  // Calculate dt from timestamp
  float dt = 1.0f / MESUREMENT_FREQUENCY;
  if (old_timestamp != -1)
  {
    if (raw_data->timestamp > (uint32_t)old_timestamp)
    {
      dt = (raw_data->timestamp - old_timestamp) / 19200000.0f;
    }
    else
    {
      // Handle timestamp overflow
      uint32_t overflow_old = 0xFFFFFFFF - old_timestamp;
      dt = (raw_data->timestamp + overflow_old) / 19200000.0f;
    }
  }
  old_timestamp = raw_data->timestamp;

  // Initialize ring buffer with first data sample to avoid zero transient
  if (!buffer_initialized)
  {
    for (int i = 0; i < LIST_SIZE; i++)
    {
      measured_accel_x[i] = raw_data->ax;
      measured_accel_y[i] = raw_data->ay;
      measured_accel_z[i] = raw_data->az;
      measured_gyro_x[i] = raw_data->gx;
      measured_gyro_y[i] = raw_data->gy;
      measured_gyro_z[i] = raw_data->gz;
    }
    buffer_initialized = true;
  }

  // Store raw data in ring buffer
  measured_accel_x[current_list_num] = raw_data->ax;
  measured_accel_y[current_list_num] = raw_data->ay;
  measured_accel_z[current_list_num] = raw_data->az;
  measured_gyro_x[current_list_num] = raw_data->gx;
  measured_gyro_y[current_list_num] = raw_data->gy;
  measured_gyro_z[current_list_num] = raw_data->gz;

  // Apply Gaussian filter
  static ImuFilteredData_t filtered_data;
  filtered_data.timestamp = raw_data->timestamp;
  filtered_data.ax = applyCausalGaussianFilter(measured_accel_x, current_list_num);
  filtered_data.ay = applyCausalGaussianFilter(measured_accel_y, current_list_num);
  filtered_data.az = applyCausalGaussianFilter(measured_accel_z, current_list_num);
  filtered_data.gx = applyCausalGaussianFilter(measured_gyro_x, current_list_num);
  filtered_data.gy = applyCausalGaussianFilter(measured_gyro_y, current_list_num);
  filtered_data.gz = applyCausalGaussianFilter(measured_gyro_z, current_list_num);
  filtered_data.dt = dt;

  current_list_num = (current_list_num + 1) % LIST_SIZE;

  // Update loop count
  loop_count++;

  // Send filtered data to MainCore
  // Send every message to maintain processing rate
  int send_ret = MP.Send(MSG_ID_FILTERED, &filtered_data, 0); // 0 = MainCore
  if (send_ret < 0)
  {
    send_error_count++;
  }

  // Send statistics every 5 seconds
  uint32_t current_time = millis();
  if (current_time - last_stats_time >= 5000)
  {
    static ProcessingStats_t stats;
    stats.core_id = 1;
    stats.loop_count = loop_count;
    stats.actual_rate_hz = (loop_count - stats_loop_count_start) / 5.0f;
    stats.dropped_messages = recv_error_count + wrong_msgid_count + send_error_count;

    MP.Send(MSG_ID_STATS, &stats, 0); // Send to MainCore

    last_stats_time = current_time;
    stats_loop_count_start = loop_count;

    // Reset error counters after reporting
    recv_error_count = 0;
    wrong_msgid_count = 0;
    send_error_count = 0;
  }
}
