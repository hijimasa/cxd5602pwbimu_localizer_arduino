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
#include "../shared_types.h"

#define LIST_SIZE 4
#define SIGMA_K 1.0f

// Ring buffer for Gaussian filter
static float measured_accel_x[LIST_SIZE];
static float measured_accel_y[LIST_SIZE];
static float measured_accel_z[LIST_SIZE];
static float measured_gyro_x[LIST_SIZE];
static float measured_gyro_y[LIST_SIZE];
static float measured_gyro_z[LIST_SIZE];
static int current_list_num = 0;
static int old_timestamp = -1;

// Gaussian kernel (initialized once)
static float kernel[LIST_SIZE];
static bool kernel_initialized = false;

void initializeKernel()
{
  float sum = 0.0f;
  for (int i = 0; i < LIST_SIZE; i++) {
    kernel[i] = (1.0f / (sqrt(2.0f * M_PI) * SIGMA_K)) *
                exp(-(float)(i * i) / (2.0f * SIGMA_K * SIGMA_K));
    sum += kernel[i];
  }
  // Normalize
  for (int i = 0; i < LIST_SIZE; i++) {
    kernel[i] /= sum;
  }
  kernel_initialized = true;
}

float applyCausalGaussianFilter(const float *x, int list_num)
{
  float y_current = 0.0f;
  for (int i = 0; i < LIST_SIZE; i++) {
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

  // Initialize ring buffers with zero
  for (int i = 0; i < LIST_SIZE; i++) {
    measured_accel_x[i] = 0.0f;
    measured_accel_y[i] = 0.0f;
    measured_accel_z[i] = 0.0f;
    measured_gyro_x[i] = 0.0f;
    measured_gyro_y[i] = 0.0f;
    measured_gyro_z[i] = 0.0f;
  }

  // Set receive timeout to blocking mode for real-time processing
  MP.RecvTimeout(MP_RECV_BLOCKING);
}

void loop()
{
  int8_t msgid;
  ImuRawData_t *raw_data;

  // Wait for raw IMU data from MainCore
  int ret = MP.Recv(&msgid, &raw_data);
  if (ret < 0) {
    return;
  }

  if (msgid != MSG_ID_IMU_RAW) {
    return;
  }

  // Calculate dt from timestamp
  float dt = 1.0f / MESUREMENT_FREQUENCY;
  if (old_timestamp != -1) {
    if (raw_data->timestamp > (uint32_t)old_timestamp) {
      dt = (raw_data->timestamp - old_timestamp) / 19200000.0f;
    } else {
      // Handle timestamp overflow
      uint32_t overflow_old = 0xFFFFFFFF - old_timestamp;
      dt = (raw_data->timestamp + overflow_old) / 19200000.0f;
    }
  }
  old_timestamp = raw_data->timestamp;

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

  // Send filtered data to SubCore2 (AHRS)
  MP.Send(MSG_ID_FILTERED, &filtered_data, SUBCORE_AHRS);
}
