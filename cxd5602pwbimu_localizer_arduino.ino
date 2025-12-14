/**
 * @file cxd5602pwbimu_localizer_ardui/home/hijikata/irlab_ws/cxd5602pwbimu_localizer_arduino/SubCore1/SubCore1.inono.ino
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

// #define VERBOSE_OUTPUT

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

  float accel_norm = sqrt(avg_accel[0] * avg_accel[0] +
                          avg_accel[1] * avg_accel[1] +
                          avg_accel[2] * avg_accel[2]);

  // Calculate earth rotation component perpendicular to gravity
  float dot_product = avg_accel[0] * avg_gyro[0] +
                      avg_accel[1] * avg_gyro[1] +
                      avg_accel[2] * avg_gyro[2];

  float earth_rot[3];
  earth_rot[0] = avg_gyro[0] - dot_product * avg_accel[0] / (accel_norm * accel_norm);
  earth_rot[1] = avg_gyro[1] - dot_product * avg_accel[1] / (accel_norm * accel_norm);
  earth_rot[2] = avg_gyro[2] - dot_product * avg_accel[2] / (accel_norm * accel_norm);

  float earth_rot_norm = sqrt(earth_rot[0] * earth_rot[0] +
                              earth_rot[1] * earth_rot[1] +
                              earth_rot[2] * earth_rot[2]);

  // Calculate coordinate axes
  float z_axis[3] = {
      avg_accel[0] / accel_norm,
      avg_accel[1] / accel_norm,
      avg_accel[2] / accel_norm};

  float y_axis[3] = {
      earth_rot[0] / earth_rot_norm,
      earth_rot[1] / earth_rot_norm,
      earth_rot[2] / earth_rot_norm};

  float x_axis[3] = {
      y_axis[1] * z_axis[2] - y_axis[2] * z_axis[1],
      y_axis[2] * z_axis[0] - y_axis[0] * z_axis[2],
      y_axis[0] * z_axis[1] - y_axis[1] * z_axis[0]};

  // Calculate quaternion from rotation matrix
  float trace = x_axis[0] + y_axis[1] + z_axis[2];
  if (trace > 0)
  {
    float s = sqrt(trace + 1.0f) * 2.0f;
    initial_quaternion[0] = 0.25f * s;
    initial_quaternion[1] = (y_axis[2] - z_axis[1]) / s;
    initial_quaternion[2] = (z_axis[0] - x_axis[2]) / s;
    initial_quaternion[3] = (x_axis[1] - y_axis[0]) / s;
  }
  else if (x_axis[0] > y_axis[1] && x_axis[0] > z_axis[2])
  {
    float s = sqrt(1.0f + x_axis[0] - y_axis[1] - z_axis[2]) * 2.0f;
    initial_quaternion[0] = (y_axis[2] - z_axis[1]) / s;
    initial_quaternion[1] = 0.25f * s;
    initial_quaternion[2] = (x_axis[1] + y_axis[0]) / s;
    initial_quaternion[3] = (z_axis[0] + x_axis[2]) / s;
  }
  else if (y_axis[1] > z_axis[2])
  {
    float s = sqrt(1.0f + y_axis[1] - x_axis[0] - z_axis[2]) * 2.0f;
    initial_quaternion[0] = (z_axis[0] - x_axis[2]) / s;
    initial_quaternion[1] = (x_axis[1] + y_axis[0]) / s;
    initial_quaternion[2] = 0.25f * s;
    initial_quaternion[3] = (y_axis[2] + z_axis[1]) / s;
  }
  else
  {
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

// 加速度バイアス初期学習関数（setup()で使用）
// 静止時の加速度を単純に累積し、後で期待される重力ベクトルを引く
void initialize_acceleration_bias(cxd5602pwbimu_data_t dat)
{
  // 生の加速度データを累積（m/s^2単位）
  acceleration_bias[0] += dat.ax;
  acceleration_bias[1] += dat.ay;
  acceleration_bias[2] += dat.az;
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
  if (cnt == 0)
    cnt = MAX_NFIFO;

  while (cnt)
  {
    read(fd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
    cnt -= MAX_NFIFO;
  }

  return 0;
}

void setup()
{
  Serial.begin(115200);
#ifdef VERBOSE_OUTPUT
  Serial.println("MainCore: Starting multicore IMU localizer...");
#endif

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

#ifdef VERBOSE_OUTPUT
  Serial.println("MainCore: Performing initial orientation calibration (5 seconds)...");
#endif

  // Initial orientation calibration
  bool is_initialized = false;
  while (!is_initialized)
  {
    ret = read(devfd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
    if (ret == sizeof(g_data[0]) * MAX_NFIFO)
    {
      for (int i = 0; i < MAX_NFIFO; i++)
      {
        is_initialized = imu_data_initialize(g_data[i]);
      }
    }
  }

  // Set initial quaternion to Fusion AHRS
  // Normalize quaternion to ensure unit length
  float quat_norm = sqrtf(initial_quaternion[0] * initial_quaternion[0] +
                          initial_quaternion[1] * initial_quaternion[1] +
                          initial_quaternion[2] * initial_quaternion[2] +
                          initial_quaternion[3] * initial_quaternion[3]);

  FusionQuaternion initialQuat;
  initialQuat.element.w = initial_quaternion[0] / quat_norm;
  initialQuat.element.x = initial_quaternion[1] / quat_norm;
  initialQuat.element.y = initial_quaternion[2] / quat_norm;
  initialQuat.element.z = initial_quaternion[3] / quat_norm;
  FusionAhrsSetQuaternion(&fusionAhrs, initialQuat);
  fusionAhrs.initialising = false;
  fusionAhrs.rampedGain = FUSION_AHRS_GAIN;

#ifdef VERBOSE_OUTPUT
  Serial.println("MainCore: Learning acceleration bias (10 seconds)...");
#endif

  // Bias learning
  while (bias_sample_count < BIAS_LEARNING_SAMPLES)
  {
    ret = read(devfd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
    if (ret == sizeof(g_data[0]) * MAX_NFIFO)
    {
      for (int i = 0; i < MAX_NFIFO; i++)
      {
        initialize_acceleration_bias(g_data[i]);
      }
    }
  }

  // Calculate average sensor acceleration
  acceleration_bias[0] /= bias_sample_count;
  acceleration_bias[1] /= bias_sample_count;
  acceleration_bias[2] /= bias_sample_count;

  // Calculate expected gravity vector in sensor frame using initial quaternion
  // In Fusion NWU convention: accelerometer measures +Z when at rest (opposing gravity)
  // So expected_accel in earth frame = [0, 0, +GRAVITY_AMOUNT]
  // We need to rotate this into sensor frame using inverse quaternion
  float qw = initial_quaternion[0];
  float qx = initial_quaternion[1];
  float qy = initial_quaternion[2];
  float qz = initial_quaternion[3];

  // Rotation matrix from Earth to Sensor frame (transpose of sensor-to-earth)
  // The quaternion represents sensor-to-earth, so we need the transpose for earth-to-sensor
  float r00 = 1.0f - 2.0f * (qy * qy + qz * qz);
  float r01 = 2.0f * (qx * qy + qw * qz);
  float r02 = 2.0f * (qx * qz - qw * qy);
  float r10 = 2.0f * (qx * qy - qw * qz);
  float r11 = 1.0f - 2.0f * (qx * qx + qz * qz);
  float r12 = 2.0f * (qy * qz + qw * qx);
  float r20 = 2.0f * (qx * qz + qw * qy);
  float r21 = 2.0f * (qy * qz - qw * qx);
  float r22 = 1.0f - 2.0f * (qx * qx + qy * qy);

  // Transform expected acceleration [0, 0, +g] from earth to sensor frame
  // g_sensor = R^T * [0, 0, +g]
  float expected_ax = r02 * GRAVITY_AMOUNT;
  float expected_ay = r12 * GRAVITY_AMOUNT;
  float expected_az = r22 * GRAVITY_AMOUNT;

  // Bias = measured - expected
  acceleration_bias[0] -= expected_ax;
  acceleration_bias[1] -= expected_ay;
  acceleration_bias[2] -= expected_az;

  bias_initialized = true;

#ifdef VERBOSE_OUTPUT
  Serial.print("MainCore: Accel bias = [");
  Serial.print(acceleration_bias[0], 4);
  Serial.print(", ");
  Serial.print(acceleration_bias[1], 4);
  Serial.print(", ");
  Serial.print(acceleration_bias[2], 4);
  Serial.println("] m/s^2");
  Serial.print("MainCore: Initial quaternion = [");
  Serial.print(initial_quaternion[0], 4);
  Serial.print(", ");
  Serial.print(initial_quaternion[1], 4);
  Serial.print(", ");
  Serial.print(initial_quaternion[2], 4);
  Serial.print(", ");
  Serial.print(initial_quaternion[3], 4);
  Serial.println("]");
#endif

#ifdef VERBOSE_OUTPUT
  Serial.println("MainCore: Starting SubCores...");
#endif

  // Start all SubCores (no longer using SubCore5 for output)
  MP.begin(SUBCORE_FILTER);
  MP.begin(SUBCORE_AHRS);
  MP.begin(SUBCORE_ZUPT);
  MP.begin(SUBCORE_POSITION);

  // Small delay to ensure SubCores are ready
  delay(100);

  // Set receive timeout to non-blocking for MainCore
  MP.RecvTimeout(MP_RECV_POLLING);

  // Send initialization parameters to SubCores
  static InitParams_t init_params;
  init_params.initial_quaternion[0] = initial_quaternion[0];
  init_params.initial_quaternion[1] = initial_quaternion[1];
  init_params.initial_quaternion[2] = initial_quaternion[2];
  init_params.initial_quaternion[3] = initial_quaternion[3];
  init_params.initial_accel_bias[0] = acceleration_bias[0];
  init_params.initial_accel_bias[1] = acceleration_bias[1];
  init_params.initial_accel_bias[2] = acceleration_bias[2];
  init_params.gyro_offset[0] = fusionOffset.gyroscopeOffset.axis.x;
  init_params.gyro_offset[1] = fusionOffset.gyroscopeOffset.axis.y;
  init_params.gyro_offset[2] = fusionOffset.gyroscopeOffset.axis.z;
  init_params.bias_initialized = bias_initialized;

  // Send to SubCore2 (AHRS), SubCore3 (ZUPT), and SubCore4 (Position)
  // Retry to ensure delivery before data processing starts
  int retry_count = 0;
  while (MP.Send(MSG_ID_INIT_COMPLETE, &init_params, SUBCORE_AHRS) < 0 && retry_count++ < 10)
  {
    delay(10);
  }
  retry_count = 0;
  while (MP.Send(MSG_ID_INIT_COMPLETE, &init_params, SUBCORE_ZUPT) < 0 && retry_count++ < 10)
  {
    delay(10);
  }
  retry_count = 0;
  while (MP.Send(MSG_ID_INIT_COMPLETE, &init_params, SUBCORE_POSITION) < 0 && retry_count++ < 10)
  {
    delay(10);
  }

  // Additional delay to ensure SubCores process initialization before data flow
  delay(50);

#ifdef VERBOSE_OUTPUT
  Serial.println("MainCore: Initialization complete. Starting data acquisition at 1920Hz.");
#endif
}

// Statistics tracking
static uint32_t last_stats_time = 0;
static uint32_t output_count = 0;
static ProcessingStats_t subcore_stats[4] = {0}; // Stats from SubCores 1-4
static uint32_t filtered_msg_count = 0;
static uint32_t ahrs_msg_count = 0;
static uint32_t zupt_msg_count = 0;

// Message relay buffers in MainCore memory space
static ImuFilteredData_t filtered_relay_buffer;
static AhrsData_t ahrs_relay_buffer;
static ZuptData_t zupt_relay_buffer;

void loop()
{
  // Process SubCore messages first to prevent queue overflow
  int8_t msgid;
  void *msgdata;

  // Process many messages from each SubCore (non-blocking)
  // Need to process ~240 msgs/sec from SubCore1
  for (int subid = 1; subid <= 4; subid++)
  {
    // Try to receive up to 20 messages from each SubCore per loop
    for (int i = 0; i < 20; i++)
    {
      int recv_ret = MP.Recv(&msgid, &msgdata, subid);

      if (recv_ret > 0)
      {
        if (msgid == MSG_ID_POSITION)
        {
          // Receive position data from SubCore4 and output via Serial
          PositionData_t *pos_data = (PositionData_t *)msgdata;

          // Output in same format as original code (hex format for float data)
          Serial.printf("%08x,%08x,%08x,%08x,"
                        "%08x,%08x,%08x,%08x,"
                        "%08x,%08x,%08x,%08x,"
                        "%08x,%08x,%08x,"
                        "%08x,%08x,%08x\n",
                        (unsigned int)pos_data->timestamp,
                        *(unsigned int *)&pos_data->temp,
                        *(unsigned int *)&pos_data->gyro[0],
                        *(unsigned int *)&pos_data->gyro[1],
                        *(unsigned int *)&pos_data->gyro[2],
                        *(unsigned int *)&pos_data->acceleration[0],
                        *(unsigned int *)&pos_data->acceleration[1],
                        *(unsigned int *)&pos_data->acceleration[2],
                        *(unsigned int *)&pos_data->quaternion[0],
                        *(unsigned int *)&pos_data->quaternion[1],
                        *(unsigned int *)&pos_data->quaternion[2],
                        *(unsigned int *)&pos_data->quaternion[3],
                        *(unsigned int *)&pos_data->velocity[0],
                        *(unsigned int *)&pos_data->velocity[1],
                        *(unsigned int *)&pos_data->velocity[2],
                        *(unsigned int *)&pos_data->position[0],
                        *(unsigned int *)&pos_data->position[1],
                        *(unsigned int *)&pos_data->position[2]);

          output_count++;
        }
        else if (msgid == MSG_ID_FILTERED)
        {
          filtered_msg_count++;
          // Copy filtered data to MainCore buffer and forward to SubCore2
          ImuFilteredData_t *filtered = (ImuFilteredData_t *)msgdata;
          filtered_relay_buffer = *filtered;
          int send_ret = MP.Send(MSG_ID_FILTERED, &filtered_relay_buffer, SUBCORE_AHRS);
          if (send_ret < 0)
          {
            // Failed to send - SubCore2 queue might be full
            static uint32_t last_error_print = 0;
            if (millis() - last_error_print > 1000)
            {
#ifdef VERBOSE_OUTPUT
              Serial.println("WARNING: Failed to send FILTERED data to SubCore2");
#endif
              last_error_print = millis();
            }
          }
        }
        else if (msgid == MSG_ID_AHRS)
        {
          ahrs_msg_count++;
          // Copy AHRS data to MainCore buffer and forward to SubCore3
          AhrsData_t *ahrs = (AhrsData_t *)msgdata;
          ahrs_relay_buffer = *ahrs;
          int send_ret = MP.Send(MSG_ID_AHRS, &ahrs_relay_buffer, SUBCORE_ZUPT);
          if (send_ret < 0)
          {
            // Failed to send - SubCore3 queue might be full
          }
        }
        else if (msgid == MSG_ID_ZUPT)
        {
          zupt_msg_count++;
          // Copy ZUPT data to MainCore buffer and forward to SubCore4
          ZuptData_t *zupt = (ZuptData_t *)msgdata;
          zupt_relay_buffer = *zupt;
          int send_ret = MP.Send(MSG_ID_ZUPT, &zupt_relay_buffer, SUBCORE_POSITION);
          if (send_ret < 0)
          {
            // Failed to send - SubCore4 queue might be full
          }
        }
        else if (msgid == MSG_ID_BIAS_DATA)
        {
          // Forward bias data from SubCore3 to SubCore2
          // Bias data is small, can forward directly
          int send_ret = MP.Send(MSG_ID_BIAS_DATA, msgdata, SUBCORE_AHRS);
          if (send_ret < 0)
          {
            // Failed to send
          }
        }
        else if (msgid == MSG_ID_STATS)
        {
          // Receive processing statistics from SubCores
          ProcessingStats_t *stats = (ProcessingStats_t *)msgdata;
          if (stats->core_id >= 1 && stats->core_id <= 4)
          {
            subcore_stats[stats->core_id - 1] = *stats;
          }
        }
      }
      else
      {
        // No more messages from this SubCore
        break;
      }
    }
  }

  // Print statistics every 5 seconds
#ifdef VERBOSE_OUTPUT
  uint32_t current_time = millis();
  if (current_time - last_stats_time >= 5000)
  {
    Serial.println("\n=== Processing Rate Statistics ===");
    Serial.printf("Output rate: %.2f Hz (expected: 30 Hz)\n",
                  output_count / 5.0f);
    Serial.printf("Message relay: FILTERED=%lu, AHRS=%lu, ZUPT=%lu\n",
                  filtered_msg_count / 5,
                  ahrs_msg_count / 5,
                  zupt_msg_count / 5);

    for (int i = 0; i < 4; i++)
    {
      if (subcore_stats[i].core_id == (i + 1))
      {
        // SubCore has sent at least one stats message
        Serial.printf("SubCore%d: %.2f Hz (loops: %lu, dropped: %lu)\n",
                      i + 1,
                      subcore_stats[i].actual_rate_hz,
                      subcore_stats[i].loop_count,
                      subcore_stats[i].dropped_messages);
      }
      else
      {
        // SubCore has never sent a stats message - not started
        Serial.printf("SubCore%d: NOT STARTED\n", i + 1);
      }
    }
    Serial.println("===================================\n");

    last_stats_time = current_time;
    output_count = 0;
    filtered_msg_count = 0;
    ahrs_msg_count = 0;
    zupt_msg_count = 0;
  }
#endif

  // Read IMU data and send to SubCore1
  ret = read(devfd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
  if (ret == sizeof(g_data[0]) * MAX_NFIFO)
  {
    for (int i = 0; i < MAX_NFIFO; i++)
    {
      // Prepare raw IMU data
      imu_raw_data.timestamp = g_data[i].timestamp;
      imu_raw_data.temp = g_data[i].temp;
      imu_raw_data.ax = g_data[i].ax;
      imu_raw_data.ay = g_data[i].ay;
      imu_raw_data.az = g_data[i].az;
      imu_raw_data.gx = g_data[i].gx;
      imu_raw_data.gy = g_data[i].gy;
      imu_raw_data.gz = g_data[i].gz;

      // Send to SubCore1
      int send_ret = MP.Send(MSG_ID_IMU_RAW, &imu_raw_data, SUBCORE_FILTER);
      if (send_ret < 0)
      {
        // Queue full, skip remaining data
        break;
      }
    }
  }
}
