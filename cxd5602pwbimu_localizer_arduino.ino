#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/cxd56_cxd5602pwbimu.h>
#include <stdbool.h>

// Fusion sensor fusion library
extern "C"
{
#include "src/Fusion/Fusion.h"
}

#define CXD5602PWBIMU_DEVPATH "/dev/imu0"
#define MAX_NFIFO (1)

#define GRAVITY_AMOUNT 9.80665f
#define EARTH_ROTATION_SPEED_AMOUNT 7.2921159e-5

#define MESUREMENT_FREQUENCY 240 // 1920
#define GYRO_NOISE_DENSITY (1.0e-3 * M_PI / 180.0f)
#define ACCEL_NOISE_DENSITY (14.0e-6 * GRAVITY_AMOUNT)
#define GYRO_NOISE_AMOUNT (GYRO_NOISE_DENSITY * sqrt(MESUREMENT_FREQUENCY))
#define ACCEL_NOISE_AMOUNT (ACCEL_NOISE_DENSITY * sqrt(MESUREMENT_FREQUENCY))
#define ACCEL_BIAS_DRIFT (4.43e-6 * GRAVITY_AMOUNT * 3.0f)
#define GYRO_BIAS_DRIFT (0.39f * M_PI / 180.0f)
// 観測ノイズの分散
#define GYRO_OBSERVATION_NOISE_VARIANCE (GYRO_NOISE_AMOUNT * GYRO_NOISE_AMOUNT)
#define ACCEL_OBSERVATION_NOISE_VARIANCE (ACCEL_NOISE_AMOUNT * ACCEL_NOISE_AMOUNT)
// プロセスノイズの分散
#define PROCESS_NOISE_VARIANCE (1.0e-7)

#define LIST_SIZE 4
#define SIGMA_K 1.0f // ガウスカーネルの標準偏差

// Fusion AHRS settings
#define FUSION_AHRS_GAIN 0.5f                                     // Recommended gain value
#define FUSION_GYRO_RANGE 2000.0f                                 // Gyroscope range in degrees/s
#define FUSION_ACCEL_REJECTION 10.0f                              // Acceleration rejection threshold in degrees
#define FUSION_RECOVERY_TRIGGER_PERIOD (5 * MESUREMENT_FREQUENCY) // 5 seconds

static cxd5602pwbimu_data_t g_data[MAX_NFIFO];
int devfd;
int ret;

// Fusion AHRS instance
FusionAhrs fusionAhrs;
FusionOffset fusionOffset;

// グローバル変数（ゼロ速度補正用）
int zero_velocity_counter = 0;

// 加速度バイアス補正用
float acceleration_bias_x = 0.0;
float acceleration_bias_y = 0.0;
float acceleration_bias_z = 0.0;
bool bias_initialized = false;
int bias_sample_count = 0;
#define BIAS_LEARNING_SAMPLES (MESUREMENT_FREQUENCY * 10) // 10 seconds of data for more stable bias estimation

bool is_initialized = false;
int current_list_num = 0;
float estimated_acceleration_x;
float estimated_acceleration_y;
float estimated_acceleration_z;
float estimated_rotation_speed_x;
float estimated_rotation_speed_y;
float estimated_rotation_speed_z;
float mesuared_acceleration_x[LIST_SIZE];
float mesuared_acceleration_y[LIST_SIZE];
float mesuared_acceleration_z[LIST_SIZE];
float mesuared_rotation_speed_x[LIST_SIZE];
float mesuared_rotation_speed_y[LIST_SIZE];
float mesuared_rotation_speed_z[LIST_SIZE];
float quaternion[4] = {1.0, 0.0, 0.0, 0.0};
float old_acceleration[3] = {0.0, 0.0, 0.0};
float velocity[3] = {0.0, 0.0, 0.0};
float old_velocity[3] = {0.0, 0.0, 0.0};
float position[3] = {0.0, 0.0, 0.0};
int old_timestamp = -1;
int calibrate_counter = 0;

// 改良版ZUPT: 加速度とジャイロの大きさのみで静止判定
// 速度は積分誤差が蓄積するため判定基準に使用しない
bool zero_velocity_correction(float accel_magnitude, float gyro_magnitude)
{
  // 静止判定の閾値
  const float ACCEL_THRESHOLD = 0.05f;                    // m/s² (重力除去後の加速度)
  const float GYRO_THRESHOLD = 0.005f;                    // rad/s (角速度、約0.86°/s)
  const int REQUIRED_SAMPLES = MESUREMENT_FREQUENCY / 10; // 0.1秒 = 192サンプル

  // 静止条件: 加速度が小さい AND ジャイロが小さい（速度は見ない！）
  if (accel_magnitude < ACCEL_THRESHOLD && gyro_magnitude < GYRO_THRESHOLD)
  {
    zero_velocity_counter++;
  }
  else
  {
    zero_velocity_counter = 0;
  }

  // 0.1秒間静止が続いたらZUPT適用
  if (zero_velocity_counter > REQUIRED_SAMPLES)
  {
    return true;
  }
  return false;
}

// 加速度バイアス初期学習関数（setup()で使用）
void initialize_acceleration_bias(cxd5602pwbimu_data_t dat)
{
  // Fusionで処理した加速度を取得
  float ax = dat.ax / GRAVITY_AMOUNT;
  float ay = dat.ay / GRAVITY_AMOUNT;
  float az = dat.az / GRAVITY_AMOUNT;

  FusionVector gyroscope = {
      dat.gx * 180.0f / M_PI,
      dat.gy * 180.0f / M_PI,
      dat.gz * 180.0f / M_PI};
  gyroscope = FusionOffsetUpdate(&fusionOffset, gyroscope);

  FusionVector accelerometer = {ax, ay, az};

  // 仮のdt（正確な値は不要）
  float dt = 1.0f / MESUREMENT_FREQUENCY;
  FusionAhrsUpdateNoMagnetometer(&fusionAhrs, gyroscope, accelerometer, dt);

  // センサ座標系の加速度を取得（重力除去済み）
  FusionVector linearAcceleration = FusionAhrsGetLinearAcceleration(&fusionAhrs);

  // バイアスを累積
  acceleration_bias_x += linearAcceleration.axis.x * GRAVITY_AMOUNT;
  acceleration_bias_y += linearAcceleration.axis.y * GRAVITY_AMOUNT;
  acceleration_bias_z += linearAcceleration.axis.z * GRAVITY_AMOUNT;
  bias_sample_count++;
}

// 加速度バイアス連続更新関数（静止時に呼び出し）
// 静止時の加速度を残差として取得し、バイアスに加算
void update_acceleration_bias(float stationary_accel_x, float stationary_accel_y, float stationary_accel_z)
{
  const float alpha = 0.01; // 学習率（1%ずつ更新）
  acceleration_bias_x += alpha * stationary_accel_x;
  acceleration_bias_y += alpha * stationary_accel_y;
  acceleration_bias_z += alpha * stationary_accel_z;
}

// 現在の時刻におけるフィルタ結果を計算（x: 入力配列、n: データ数、kernel: ガウスカーネル、K: カーネルサイズ）
float apply_causal_gaussian_filter(const float *x, int list_num)
{
  static float kernel[LIST_SIZE];
  static bool is_kernel_initialized = false;
  if (!is_kernel_initialized)
  {
    float sum = 0.0;
    for (int i = 0; i < LIST_SIZE; i++)
    {
      kernel[i] = (1.0 / (sqrt(2.0 * M_PI) * SIGMA_K)) * exp(-(i * i) / (2.0 * SIGMA_K * SIGMA_K));
      sum += kernel[i];
    }
    // 正規化
    for (int i = 0; i < LIST_SIZE; i++)
    {
      kernel[i] /= sum;
    }

    is_kernel_initialized = true;
  }

  float y_current = 0.0;
  // 最新のデータが x[n-1] とし、過去方向にカーネルを適用
  for (int i = 0; i < LIST_SIZE; i++)
  {
    int idx = LIST_SIZE - 1 - i;
    int list_idx = (list_num + idx) % LIST_SIZE;
    y_current += kernel[i] * x[list_idx];
  }
  return y_current;
}

bool imu_data_initialize(cxd5602pwbimu_data_t dat)
{
  static int initialize_counter = 0;
  static float initialize_acceleration_x_sum = 0.0;
  static float initialize_acceleration_y_sum = 0.0;
  static float initialize_acceleration_z_sum = 0.0;
  static float initialize_rotation_speed_x_sum = 0.0;
  static float initialize_rotation_speed_y_sum = 0.0;
  static float initialize_rotation_speed_z_sum = 0.0;
  if (initialize_counter < MESUREMENT_FREQUENCY * 5) // 5秒間データを蓄積
  {
    initialize_acceleration_x_sum += dat.ax;
    initialize_acceleration_y_sum += dat.ay;
    initialize_acceleration_z_sum += dat.az;
    initialize_rotation_speed_x_sum += dat.gx;
    initialize_rotation_speed_y_sum += dat.gy;
    initialize_rotation_speed_z_sum += dat.gz;
    initialize_counter++;

    return false;
  }
  else
  {
    estimated_acceleration_x = initialize_acceleration_x_sum / (MESUREMENT_FREQUENCY * 5);
    estimated_acceleration_y = initialize_acceleration_y_sum / (MESUREMENT_FREQUENCY * 5);
    estimated_acceleration_z = initialize_acceleration_z_sum / (MESUREMENT_FREQUENCY * 5);
    float average_acceleration_norm = sqrt(estimated_acceleration_x * estimated_acceleration_x +
                                           estimated_acceleration_y * estimated_acceleration_y +
                                           estimated_acceleration_z * estimated_acceleration_z);
    estimated_rotation_speed_x = initialize_rotation_speed_x_sum / (MESUREMENT_FREQUENCY * 5);
    estimated_rotation_speed_y = initialize_rotation_speed_y_sum / (MESUREMENT_FREQUENCY * 5);
    estimated_rotation_speed_z = initialize_rotation_speed_z_sum / (MESUREMENT_FREQUENCY * 5);
    float dot_product = estimated_acceleration_x * estimated_rotation_speed_x +
                        estimated_acceleration_y * estimated_rotation_speed_y +
                        estimated_acceleration_z * estimated_rotation_speed_z;
    float earth_rotation_speed_x = estimated_rotation_speed_x - dot_product * estimated_acceleration_x / average_acceleration_norm / average_acceleration_norm;
    float earth_rotation_speed_y = estimated_rotation_speed_y - dot_product * estimated_acceleration_y / average_acceleration_norm / average_acceleration_norm;
    float earth_rotation_speed_z = estimated_rotation_speed_z - dot_product * estimated_acceleration_z / average_acceleration_norm / average_acceleration_norm;
    float earth_speed_norm = sqrt(earth_rotation_speed_x * earth_rotation_speed_x +
                                  earth_rotation_speed_y * earth_rotation_speed_y +
                                  earth_rotation_speed_z * earth_rotation_speed_z);
    initialize_counter = 0;
    initialize_acceleration_x_sum = 0.0f;
    initialize_acceleration_y_sum = 0.0f;
    initialize_acceleration_z_sum = 0.0f;
    initialize_rotation_speed_x_sum = 0.0f;
    initialize_rotation_speed_y_sum = 0.0f;
    initialize_rotation_speed_z_sum = 0.0f;

    for (int i = 0; i < LIST_SIZE; i++)
    {
      mesuared_acceleration_x[i] = estimated_acceleration_x;
      mesuared_acceleration_y[i] = estimated_acceleration_y;
      mesuared_acceleration_z[i] = estimated_acceleration_z;
      mesuared_rotation_speed_x[i] = estimated_rotation_speed_x;
      mesuared_rotation_speed_y[i] = estimated_rotation_speed_y;
      mesuared_rotation_speed_z[i] = estimated_rotation_speed_z;
    }

    // 単位ベクトル計算
    float z_axis_x = estimated_acceleration_x / average_acceleration_norm;
    float z_axis_y = estimated_acceleration_y / average_acceleration_norm;
    float z_axis_z = estimated_acceleration_z / average_acceleration_norm;
    float y_axis_x = earth_rotation_speed_x / earth_speed_norm;
    float y_axis_y = earth_rotation_speed_y / earth_speed_norm;
    float y_axis_z = earth_rotation_speed_z / earth_speed_norm;
    float x_axis_x = y_axis_y * z_axis_z - y_axis_z * z_axis_y;
    float x_axis_y = y_axis_z * z_axis_x - y_axis_x * z_axis_z;
    float x_axis_z = y_axis_x * z_axis_y - y_axis_y * z_axis_x;
    float trace = x_axis_x + y_axis_y + z_axis_z;
    if (trace > 0)
    {
      float s = sqrt(trace + 1.0) * 2.0;
      quaternion[0] = 0.25 * s;
      quaternion[1] = (y_axis_z - z_axis_y) / s;
      quaternion[2] = (z_axis_x - x_axis_z) / s;
      quaternion[3] = (x_axis_y - y_axis_x) / s;
    }
    else if (x_axis_x > y_axis_y && x_axis_x > z_axis_z)
    {
      float s = sqrt(1.0 + x_axis_x - y_axis_y - z_axis_z) * 2.0;
      quaternion[0] = (y_axis_z - z_axis_y) / s;
      quaternion[1] = 0.25 * s;
      quaternion[2] = (x_axis_y + y_axis_x) / s;
      quaternion[3] = (z_axis_x + x_axis_z) / s;
    }
    else if (y_axis_y > z_axis_z)
    {
      float s = sqrt(1.0 + y_axis_y - x_axis_x - z_axis_z) * 2.0;
      quaternion[0] = (z_axis_x - x_axis_z) / s;
      quaternion[1] = (x_axis_y + y_axis_x) / s;
      quaternion[2] = 0.25 * s;
      quaternion[3] = (y_axis_z + z_axis_y) / s;
    }
    else
    {
      float s = sqrt(1.0 + z_axis_z - x_axis_x - y_axis_y) * 2.0;
      quaternion[0] = (x_axis_y - y_axis_x) / s;
      quaternion[1] = (z_axis_x + x_axis_z) / s;
      quaternion[2] = (y_axis_z + z_axis_y) / s;
      quaternion[3] = 0.25 * s;
    }
    return true;
  }
  return false;
}

void update(cxd5602pwbimu_data_t dat)
{
  float dt = 1.0 / MESUREMENT_FREQUENCY;
  if (old_timestamp == -1)
  {
    old_timestamp = dat.timestamp;
  }
  else
  {
    if (dat.timestamp > old_timestamp)
    {
      dt = (dat.timestamp - old_timestamp) / 19200000.0f;
    }
    else
    {
      old_timestamp = 0xFFFFFFFF - old_timestamp;
      dt = (dat.timestamp + old_timestamp) / 19200000.0f;
    }
    old_timestamp = dat.timestamp;
  }

  mesuared_acceleration_x[current_list_num] = dat.ax;
  mesuared_acceleration_y[current_list_num] = dat.ay;
  mesuared_acceleration_z[current_list_num] = dat.az;
  mesuared_rotation_speed_x[current_list_num] = dat.gx;
  mesuared_rotation_speed_y[current_list_num] = dat.gy;
  mesuared_rotation_speed_z[current_list_num] = dat.gz;

  // ガウスフィルタを適用（ノイズ低減）
  estimated_acceleration_x = apply_causal_gaussian_filter(mesuared_acceleration_x, current_list_num);
  estimated_acceleration_y = apply_causal_gaussian_filter(mesuared_acceleration_y, current_list_num);
  estimated_acceleration_z = apply_causal_gaussian_filter(mesuared_acceleration_z, current_list_num);
  estimated_rotation_speed_x = apply_causal_gaussian_filter(mesuared_rotation_speed_x, current_list_num);
  estimated_rotation_speed_y = apply_causal_gaussian_filter(mesuared_rotation_speed_y, current_list_num);
  estimated_rotation_speed_z = apply_causal_gaussian_filter(mesuared_rotation_speed_z, current_list_num);

    // バイアス補正を適用（初期化済みの場合）
  if (bias_initialized)
  {
    estimated_acceleration_x -= acceleration_bias_x;
    estimated_acceleration_y -= acceleration_bias_y;
    estimated_acceleration_z -= acceleration_bias_z;
  }

  // Apply gyroscope offset correction (runtime bias estimation)
  FusionVector gyroscope = {
      estimated_rotation_speed_x * 180.0f / M_PI, // Convert rad/s to deg/s
      estimated_rotation_speed_y * 180.0f / M_PI,
      estimated_rotation_speed_z * 180.0f / M_PI};
  gyroscope = FusionOffsetUpdate(&fusionOffset, gyroscope);

  // Prepare accelerometer data (convert to g units)
  FusionVector accelerometer = {
      estimated_acceleration_x / GRAVITY_AMOUNT,
      estimated_acceleration_y / GRAVITY_AMOUNT,
      estimated_acceleration_z / GRAVITY_AMOUNT};

  // Update Fusion AHRS (without magnetometer)
  FusionAhrsUpdateNoMagnetometer(&fusionAhrs, gyroscope, accelerometer, dt);

  // Get quaternion from Fusion AHRS
  FusionQuaternion fusionQuaternion = FusionAhrsGetQuaternion(&fusionAhrs);
  quaternion[0] = fusionQuaternion.element.w;
  quaternion[1] = fusionQuaternion.element.x;
  quaternion[2] = fusionQuaternion.element.y;
  quaternion[3] = fusionQuaternion.element.z;

  // Get linear acceleration (gravity removed, in sensor frame)
  FusionVector linearAcceleration = FusionAhrsGetLinearAcceleration(&fusionAhrs);
  float sensor_based_acceleration_x = linearAcceleration.axis.x * GRAVITY_AMOUNT;
  float sensor_based_acceleration_y = linearAcceleration.axis.y * GRAVITY_AMOUNT;
  float sensor_based_acceleration_z = linearAcceleration.axis.z * GRAVITY_AMOUNT;

  // Get earth acceleration (gravity already removed, in g units)
  FusionVector earthAcceleration = FusionAhrsGetEarthAcceleration(&fusionAhrs);

  // Convert from g to m/s²
  estimated_acceleration_x = earthAcceleration.axis.x * GRAVITY_AMOUNT;
  estimated_acceleration_y = earthAcceleration.axis.y * GRAVITY_AMOUNT;
  estimated_acceleration_z = earthAcceleration.axis.z * GRAVITY_AMOUNT;

  // Note: Fusion AHRS automatically handles:
  // - Quaternion update with gyroscope integration
  // - Accelerometer correction with intelligent rejection during motion
  // - Gravity vector tracking and removal
  // - Coordinate frame transformations
  // This replaces the old Madgwick filter + RK4 integration


  // 加速度とジャイロの大きさを計算（ZUPT判定用）
  float accel_magnitude = sqrt(estimated_acceleration_x * estimated_acceleration_x +
                               estimated_acceleration_y * estimated_acceleration_y +
                               estimated_acceleration_z * estimated_acceleration_z);
  float gyro_magnitude = sqrt(estimated_rotation_speed_x * estimated_rotation_speed_x +
                              estimated_rotation_speed_y * estimated_rotation_speed_y +
                              estimated_rotation_speed_z * estimated_rotation_speed_z);

  // 速度の指数減衰モデル（動きが小さいほど速度を減衰）
  // センサの動きの大きさを統合指標として計算
  float motion_magnitude = accel_magnitude + gyro_magnitude * 5.0f; // ジャイロにスケール係数を適用

  // 減衰係数の計算（motion_magnitudeが小さいほど強く減衰）
  // motion_magnitude < 0.1 の範囲で、0.90 ～ 0.999 まで変化
  const float MOTION_THRESHOLD = 0.1f; // この値以下で減衰を適用
  const float MIN_DECAY = 0.90f;       // 最大減衰時（静止時）
  const float MAX_DECAY = 0.999f;      // 最小減衰時（動作時）

  float decay_factor = 1.0f;
  if (motion_magnitude < MOTION_THRESHOLD)
  {
    // 0.0 → MIN_DECAY (0.90), MOTION_THRESHOLD → MAX_DECAY (0.999)
    float normalized_motion = motion_magnitude / MOTION_THRESHOLD;
    decay_factor = MIN_DECAY + (MAX_DECAY - MIN_DECAY) * normalized_motion;

    // 速度に減衰を適用
    velocity[0] *= decay_factor;
    velocity[1] *= decay_factor;
    velocity[2] *= decay_factor;
  }

  // 速度・位置の更新（台形積分）
  velocity[0] += (estimated_acceleration_x + old_acceleration[0]) / 2.0f * dt;
  velocity[1] += (estimated_acceleration_y + old_acceleration[1]) / 2.0f * dt;
  velocity[2] += (estimated_acceleration_z + old_acceleration[2]) / 2.0f * dt;
  position[0] += (velocity[0] + old_velocity[0]) / 2.0f * dt;
  position[1] += (velocity[1] + old_velocity[1]) / 2.0f * dt;
  position[2] += (velocity[2] + old_velocity[2]) / 2.0f * dt;

  // 改良版ZUPT判定と適用（速度は見ない、センサーの生データのみ）
  bool is_stationary = zero_velocity_correction(accel_magnitude, gyro_magnitude);

  if (is_stationary)
  {
    // 速度をゼロにリセット
    velocity[0] = 0.0;
    velocity[1] = 0.0;
    velocity[2] = 0.0;

    // 静止時はバイアスを連続更新（温度ドリフト等に追従）
    if (bias_initialized)
    {
      update_acceleration_bias(sensor_based_acceleration_x, sensor_based_acceleration_y, sensor_based_acceleration_z);
    }
  }

  old_acceleration[0] = estimated_acceleration_x;
  old_acceleration[1] = estimated_acceleration_y;
  old_acceleration[2] = estimated_acceleration_z;
  old_velocity[0] = velocity[0];
  old_velocity[1] = velocity[1];
  old_velocity[2] = velocity[2];

  current_list_num = (current_list_num + 1) % LIST_SIZE;

  return;
}

static int start_sensing(int fd, int rate, int adrange, int gdrange,
                         int nfifos)
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
  int cnt = samprate / 20; /* data size of 50ms */

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

  board_cxd5602pwbimu_initialize(5);

  devfd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);

  start_sensing(devfd, MESUREMENT_FREQUENCY, 8, 2000, MAX_NFIFO);
  drop_50msdata(devfd, MESUREMENT_FREQUENCY);

  // Initialize Fusion AHRS
  FusionOffsetInitialise(&fusionOffset, MESUREMENT_FREQUENCY);
  FusionAhrsInitialise(&fusionAhrs);

  // Configure Fusion AHRS settings
  const FusionAhrsSettings settings = {
      .convention = FusionConventionNwu, // North-West-Up convention
      .gain = FUSION_AHRS_GAIN,
      .gyroscopeRange = FUSION_GYRO_RANGE,
      .accelerationRejection = FUSION_ACCEL_REJECTION,
      .magneticRejection = 0.0f, // No magnetometer
      .recoveryTriggerPeriod = FUSION_RECOVERY_TRIGGER_PERIOD,
  };
  FusionAhrsSetSettings(&fusionAhrs, &settings);

  // dump_data(devfd);
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

  // 計算された初期姿勢をFusion AHRSに設定
  FusionQuaternion initialQuaternion;
  initialQuaternion.element.w = quaternion[0];
  initialQuaternion.element.x = quaternion[1];
  initialQuaternion.element.y = quaternion[2];
  initialQuaternion.element.z = quaternion[3];
  FusionAhrsSetQuaternion(&fusionAhrs, initialQuaternion);

  fusionAhrs.initialising = false;
  fusionAhrs.rampedGain = FUSION_AHRS_GAIN;

  // 加速度バイアスの初期学習（10秒間、19200サンプル）
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

  // 平均を取る
  acceleration_bias_x /= bias_sample_count;
  acceleration_bias_y /= bias_sample_count;
  acceleration_bias_z /= bias_sample_count;
  bias_initialized = true;
}

void loop()
{
  static int execute_counter = 0;
  ret = read(devfd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
  if (ret == sizeof(g_data[0]) * MAX_NFIFO)
  {
    for (int i = 0; i < MAX_NFIFO; i++)
    {
      update(g_data[i]);

      execute_counter++;
      if (execute_counter >= MESUREMENT_FREQUENCY / 30)
      {
        Serial.printf("%08x,%08x,%08x,%08x,"
                      "%08x,%08x,%08x,%08x,"
                      "%08x,%08x,%08x,%08x,"
                      "%08x,%08x,%08x,"
                      "%08x,%08x,%08x\n",
                      (unsigned int)g_data[i].timestamp,
                      *(unsigned int *)&g_data[i].temp,
                      *(unsigned int *)&estimated_rotation_speed_x,
                      *(unsigned int *)&estimated_rotation_speed_y,
                      *(unsigned int *)&estimated_rotation_speed_z,
                      *(unsigned int *)&estimated_acceleration_x,
                      *(unsigned int *)&estimated_acceleration_y,
                      *(unsigned int *)&estimated_acceleration_z,
                      *(unsigned int *)&quaternion[0],
                      *(unsigned int *)&quaternion[1],
                      *(unsigned int *)&quaternion[2],
                      *(unsigned int *)&quaternion[3],
                      *(unsigned int *)&velocity[0],
                      *(unsigned int *)&velocity[1],
                      *(unsigned int *)&velocity[2],
                      *(unsigned int *)&position[0],
                      *(unsigned int *)&position[1],
                      *(unsigned int *)&position[2]);
        execute_counter = 0;
      }
    }
  }
}