/**
 * @file cxd5602pwbimu_localizer_arduino.ino
 * @brief MainCore - IMU Data Acquisition and Distribution
 *
 * MainCore: Reads IMU data at 1920Hz, performs initial calibration,
 * and distributes data to SubCore1 (Gaussian Filter)
 *
 * Multicore Architecture:
 *   MainCore -> SubCore1 (Filter) -> SubCore2 (AHRS) ->
 *   SubCore3 (ZUPT) -> SubCore4 (Position) -> SubCore5 (Output)
 */

#ifdef SUBCORE
#error "Core selection is wrong!! Must select MainCore"
#endif

#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/cxd56_cxd5602pwbimu.h>
#include <stdbool.h>
#include <MP.h>

// Fusion sensor fusion library
extern "C"
{
#include "src/Fusion/Fusion.h"
}

#include "shared_types.h"

#define CXD5602PWBIMU_DEVPATH "/dev/imu0"
#define MAX_NFIFO (1)

static cxd5602pwbimu_data_t g_data[MAX_NFIFO];
int devfd;
int ret;

// Fusion AHRS instance (for initialization only)
FusionAhrs fusionAhrs;
FusionOffset fusionOffset;

// Initialization state
float initial_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float acceleration_bias[3] = {0.0f, 0.0f, 0.0f};
bool bias_initialized = false;
int bias_sample_count = 0;

#define BIAS_LEARNING_SAMPLES (MESUREMENT_FREQUENCY * 10)

// Shared data buffer for IMU raw data
static ImuRawData_t imu_raw_data;

// Initialization variables
static float init_accel_sum[3] = {0.0f, 0.0f, 0.0f};
static float init_gyro_sum[3] = {0.0f, 0.0f, 0.0f};
static int init_counter = 0;

bool imu_data_initialize(cxd5602pwbimu_data_t dat)
{
  if (init_counter < MESUREMENT_FREQUENCY * 5) // 5 seconds
  {
    init_accel_sum[0] += dat.ax;
    init_accel_sum[1] += dat.ay;
    init_accel_sum[2] += dat.az;
    init_gyro_sum[0] += dat.gx;
    init_gyro_sum[1] += dat.gy;
    init_gyro_sum[2] += dat.gz;
    init_counter++;
    return false;
  }

  // Calculate averages
  float avg_accel[3], avg_gyro[3];
  int sample_count = MESUREMENT_FREQUENCY * 5;
  avg_accel[0] = init_accel_sum[0] / sample_count;
  avg_accel[1] = init_accel_sum[1] / sample_count;
  avg_accel[2] = init_accel_sum[2] / sample_count;
  avg_gyro[0] = init_gyro_sum[0] / sample_count;
  avg_gyro[1] = init_gyro_sum[1] / sample_count;
  avg_gyro[2] = init_gyro_sum[2] / sample_count;

  float accel_norm = sqrt(avg_accel[0]*avg_accel[0] +
                          avg_accel[1]*avg_accel[1] +
                          avg_accel[2]*avg_accel[2]);

  // Calculate earth rotation component perpendicular to gravity
  float dot_product = avg_accel[0]*avg_gyro[0] +
                      avg_accel[1]*avg_gyro[1] +
                      avg_accel[2]*avg_gyro[2];

  float earth_rot[3];
  earth_rot[0] = avg_gyro[0] - dot_product * avg_accel[0] / (accel_norm * accel_norm);
  earth_rot[1] = avg_gyro[1] - dot_product * avg_accel[1] / (accel_norm * accel_norm);
  earth_rot[2] = avg_gyro[2] - dot_product * avg_accel[2] / (accel_norm * accel_norm);

  float earth_rot_norm = sqrt(earth_rot[0]*earth_rot[0] +
                              earth_rot[1]*earth_rot[1] +
                              earth_rot[2]*earth_rot[2]);

  // Calculate coordinate axes
  float z_axis[3] = {
    avg_accel[0] / accel_norm,
    avg_accel[1] / accel_norm,
    avg_accel[2] / accel_norm
  };

  float y_axis[3] = {
    earth_rot[0] / earth_rot_norm,
    earth_rot[1] / earth_rot_norm,
    earth_rot[2] / earth_rot_norm
  };

  float x_axis[3] = {
    y_axis[1]*z_axis[2] - y_axis[2]*z_axis[1],
    y_axis[2]*z_axis[0] - y_axis[0]*z_axis[2],
    y_axis[0]*z_axis[1] - y_axis[1]*z_axis[0]
  };

  // Calculate quaternion from rotation matrix
  float trace = x_axis[0] + y_axis[1] + z_axis[2];
  if (trace > 0) {
    float s = sqrt(trace + 1.0f) * 2.0f;
    initial_quaternion[0] = 0.25f * s;
    initial_quaternion[1] = (y_axis[2] - z_axis[1]) / s;
    initial_quaternion[2] = (z_axis[0] - x_axis[2]) / s;
    initial_quaternion[3] = (x_axis[1] - y_axis[0]) / s;
  } else if (x_axis[0] > y_axis[1] && x_axis[0] > z_axis[2]) {
    float s = sqrt(1.0f + x_axis[0] - y_axis[1] - z_axis[2]) * 2.0f;
    initial_quaternion[0] = (y_axis[2] - z_axis[1]) / s;
    initial_quaternion[1] = 0.25f * s;
    initial_quaternion[2] = (x_axis[1] + y_axis[0]) / s;
    initial_quaternion[3] = (z_axis[0] + x_axis[2]) / s;
  } else if (y_axis[1] > z_axis[2]) {
    float s = sqrt(1.0f + y_axis[1] - x_axis[0] - z_axis[2]) * 2.0f;
    initial_quaternion[0] = (z_axis[0] - x_axis[2]) / s;
    initial_quaternion[1] = (x_axis[1] + y_axis[0]) / s;
    initial_quaternion[2] = 0.25f * s;
    initial_quaternion[3] = (y_axis[2] + z_axis[1]) / s;
  } else {
    float s = sqrt(1.0f + z_axis[2] - x_axis[0] - y_axis[1]) * 2.0f;
    initial_quaternion[0] = (x_axis[1] - y_axis[0]) / s;
    initial_quaternion[1] = (z_axis[0] + x_axis[2]) / s;
    initial_quaternion[2] = (y_axis[2] + z_axis[1]) / s;
    initial_quaternion[3] = 0.25f * s;
  }

  // Reset for potential future use
  init_counter = 0;
  init_accel_sum[0] = init_accel_sum[1] = init_accel_sum[2] = 0.0f;
  init_gyro_sum[0] = init_gyro_sum[1] = init_gyro_sum[2] = 0.0f;

  return true;
}

void initialize_acceleration_bias(cxd5602pwbimu_data_t dat)
{
  float ax = dat.ax / GRAVITY_AMOUNT;
  float ay = dat.ay / GRAVITY_AMOUNT;
  float az = dat.az / GRAVITY_AMOUNT;

  FusionVector gyroscope = {
    dat.gx * 180.0f / (float)M_PI,
    dat.gy * 180.0f / (float)M_PI,
    dat.gz * 180.0f / (float)M_PI
  };
  gyroscope = FusionOffsetUpdate(&fusionOffset, gyroscope);

  FusionVector accelerometer = {ax, ay, az};

  float dt = 1.0f / MESUREMENT_FREQUENCY;
  FusionAhrsUpdateNoMagnetometer(&fusionAhrs, gyroscope, accelerometer, dt);

  FusionVector linearAccel = FusionAhrsGetLinearAcceleration(&fusionAhrs);

  acceleration_bias[0] += linearAccel.axis.x * GRAVITY_AMOUNT;
  acceleration_bias[1] += linearAccel.axis.y * GRAVITY_AMOUNT;
  acceleration_bias[2] += linearAccel.axis.z * GRAVITY_AMOUNT;
  bias_sample_count++;
}

static int start_sensing(int fd, int rate, int adrange, int gdrange, int nfifos)
{
  cxd5602pwbimu_range_t range;

  ioctl(fd, SNIOC_SSAMPRATE, rate);
  range.accel = adrange;
  range.gyro = gdrange;
  ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  ioctl(fd, SNIOC_SFIFOTHRESH, nfifos);
  ioctl(fd, SNIOC_ENABLE, 1);

  return 0;
}

static int drop_50msdata(int fd, int samprate)
{
  int cnt = samprate / 20;
  cnt = ((cnt + MAX_NFIFO - 1) / MAX_NFIFO) * MAX_NFIFO;
  if (cnt == 0) cnt = MAX_NFIFO;

  while (cnt) {
    read(fd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
    cnt -= MAX_NFIFO;
  }

  return 0;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("MainCore: Starting multicore IMU localizer...");

  // Initialize IMU hardware
  board_cxd5602pwbimu_initialize(5);
  devfd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);

  start_sensing(devfd, MESUREMENT_FREQUENCY, 8, 2000, MAX_NFIFO);
  drop_50msdata(devfd, MESUREMENT_FREQUENCY);

  // Initialize Fusion AHRS (for bias learning)
  FusionOffsetInitialise(&fusionOffset, MESUREMENT_FREQUENCY);
  FusionAhrsInitialise(&fusionAhrs);

  const FusionAhrsSettings settings = {
    .convention = FusionConventionNwu,
    .gain = FUSION_AHRS_GAIN,
    .gyroscopeRange = FUSION_GYRO_RANGE,
    .accelerationRejection = FUSION_ACCEL_REJECTION,
    .magneticRejection = 0.0f,
    .recoveryTriggerPeriod = FUSION_RECOVERY_TRIGGER_PERIOD,
  };
  FusionAhrsSetSettings(&fusionAhrs, &settings);

  Serial.println("MainCore: Performing initial orientation calibration (5 seconds)...");

  // Initial orientation calibration
  bool is_initialized = false;
  while (!is_initialized) {
    ret = read(devfd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
    if (ret == sizeof(g_data[0]) * MAX_NFIFO) {
      for (int i = 0; i < MAX_NFIFO; i++) {
        is_initialized = imu_data_initialize(g_data[i]);
      }
    }
  }

  // Set initial quaternion to Fusion AHRS
  FusionQuaternion initialQuat;
  initialQuat.element.w = initial_quaternion[0];
  initialQuat.element.x = initial_quaternion[1];
  initialQuat.element.y = initial_quaternion[2];
  initialQuat.element.z = initial_quaternion[3];
  FusionAhrsSetQuaternion(&fusionAhrs, initialQuat);
  fusionAhrs.initialising = false;
  fusionAhrs.rampedGain = FUSION_AHRS_GAIN;

  Serial.println("MainCore: Learning acceleration bias (10 seconds)...");

  // Bias learning
  while (bias_sample_count < BIAS_LEARNING_SAMPLES) {
    ret = read(devfd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
    if (ret == sizeof(g_data[0]) * MAX_NFIFO) {
      for (int i = 0; i < MAX_NFIFO; i++) {
        initialize_acceleration_bias(g_data[i]);
      }
    }
  }

  acceleration_bias[0] /= bias_sample_count;
  acceleration_bias[1] /= bias_sample_count;
  acceleration_bias[2] /= bias_sample_count;
  bias_initialized = true;

  Serial.println("MainCore: Starting SubCores...");

  // Start all SubCores
  MP.begin(SUBCORE_FILTER);
  MP.begin(SUBCORE_AHRS);
  MP.begin(SUBCORE_ZUPT);
  MP.begin(SUBCORE_POSITION);
  MP.begin(SUBCORE_OUTPUT);

  // Small delay to ensure SubCores are ready
  delay(100);

  // Send initialization parameters to SubCores
  static InitParams_t init_params;
  init_params.initial_quaternion[0] = initial_quaternion[0];
  init_params.initial_quaternion[1] = initial_quaternion[1];
  init_params.initial_quaternion[2] = initial_quaternion[2];
  init_params.initial_quaternion[3] = initial_quaternion[3];
  init_params.initial_accel_bias[0] = acceleration_bias[0];
  init_params.initial_accel_bias[1] = acceleration_bias[1];
  init_params.initial_accel_bias[2] = acceleration_bias[2];
  init_params.bias_initialized = bias_initialized;

  // Send to SubCore2 (AHRS) and SubCore3 (ZUPT)
  MP.Send(MSG_ID_INIT_COMPLETE, &init_params, SUBCORE_AHRS);
  MP.Send(MSG_ID_INIT_COMPLETE, &init_params, SUBCORE_ZUPT);

  Serial.println("MainCore: Initialization complete. Starting data acquisition at 1920Hz.");
}

void loop()
{
  ret = read(devfd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
  if (ret == sizeof(g_data[0]) * MAX_NFIFO) {
    for (int i = 0; i < MAX_NFIFO; i++) {
      // Prepare raw IMU data
      imu_raw_data.timestamp = g_data[i].timestamp;
      imu_raw_data.temp = g_data[i].temp;
      imu_raw_data.ax = g_data[i].ax;
      imu_raw_data.ay = g_data[i].ay;
      imu_raw_data.az = g_data[i].az;
      imu_raw_data.gx = g_data[i].gx;
      imu_raw_data.gy = g_data[i].gy;
      imu_raw_data.gz = g_data[i].gz;

      // Send to SubCore1 (Gaussian Filter)
      MP.Send(MSG_ID_IMU_RAW, &imu_raw_data, SUBCORE_FILTER);
    }
  }
}
