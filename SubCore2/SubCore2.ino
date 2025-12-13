/**c:\Users\hijim\OneDrive\Documents\cxd5602pwbimu_localizer_arduino\SubCore3\SubCore3.ino
 * @file SubCore2.ino
 * @brief Fusion AHRS Processing Core
 *
 * SubCore2: Receives filtered IMU data, performs AHRS computation
 * using Fusion library, then forwards to SubCore3 (ZUPT)
 */

#if (SUBCORE != 2)
#error "Core selection is wrong!! Must select SubCore2"
#endif

#include <MP.h>
#include "shared_types.h"

// Fusion library
extern "C"
{
#include "src/Fusion/Fusion.h"
}

// Fusion AHRS instances
static FusionAhrs fusionAhrs;
static FusionOffset fusionOffset;
static bool ahrs_initialized = false;

// Bias data received from MainCore
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};
static bool bias_initialized = false;
static bool init_complete = false; // Wait for initialization from MainCore

// Statistics tracking
static uint32_t loop_count = 0;
static uint32_t last_stats_time = 0;
static uint32_t stats_loop_count_start = 0;
static uint32_t recv_error_count = 0;
static uint32_t wrong_msgid_count = 0;
static uint32_t total_loop_entry_count = 0;

void setup()
{
  // Initialize MP library
  MP.begin();

  // Set receive timeout to blocking mode (like other SubCores)
  MP.RecvTimeout(MP_RECV_BLOCKING);

  last_stats_time = millis();

  // Initialize Fusion AHRS
  FusionOffsetInitialise(&fusionOffset, MESUREMENT_FREQUENCY);
  FusionAhrsInitialise(&fusionAhrs);

  // Configure Fusion AHRS settings
  const FusionAhrsSettings settings = {
      .convention = FusionConventionNwu,
      .gain = FUSION_AHRS_GAIN,
      .gyroscopeRange = FUSION_GYRO_RANGE,
      .accelerationRejection = FUSION_ACCEL_REJECTION,
      .magneticRejection = 0.0f,
      .recoveryTriggerPeriod = FUSION_RECOVERY_TRIGGER_PERIOD,
  };
  FusionAhrsSetSettings(&fusionAhrs, &settings);

  ahrs_initialized = true;

  // Send startup notification to MainCore
  static ProcessingStats_t startup_stats;
  startup_stats.core_id = 2;
  startup_stats.loop_count = 999; // Special value to indicate setup() completed
  startup_stats.actual_rate_hz = 0.0f;
  startup_stats.dropped_messages = 0;

  MP.Send(MSG_ID_STATS, &startup_stats, 0);
}

void loop()
{
  total_loop_entry_count++;

  int8_t msgid;
  void *msgdata;

  // Try to receive message (non-blocking)
  int ret = MP.Recv(&msgid, &msgdata);
  if (ret < 0)
  {
    // No message available - this is normal in non-blocking mode
    return;
  }

  // Handle initialization message from MainCore
  if (msgid == MSG_ID_INIT_COMPLETE)
  {
    InitParams_t *init_params = (InitParams_t *)msgdata;

    // Set initial quaternion
    FusionQuaternion initialQuat;
    initialQuat.element.w = init_params->initial_quaternion[0];
    initialQuat.element.x = init_params->initial_quaternion[1];
    initialQuat.element.y = init_params->initial_quaternion[2];
    initialQuat.element.z = init_params->initial_quaternion[3];
    FusionAhrsSetQuaternion(&fusionAhrs, initialQuat);

    fusionAhrs.initialising = false;
    fusionAhrs.rampedGain = FUSION_AHRS_GAIN;

    // Set bias
    accel_bias[0] = init_params->initial_accel_bias[0];
    accel_bias[1] = init_params->initial_accel_bias[1];
    accel_bias[2] = init_params->initial_accel_bias[2];

    // Set gyro offset from MainCore's learning
    fusionOffset.gyroscopeOffset.axis.x = init_params->gyro_offset[0];
    fusionOffset.gyroscopeOffset.axis.y = init_params->gyro_offset[1];
    fusionOffset.gyroscopeOffset.axis.z = init_params->gyro_offset[2];
    bias_initialized = init_params->bias_initialized;
    init_complete = true; // Now ready to process data

    return;
  }

  // Handle bias update from SubCore3
  if (msgid == MSG_ID_BIAS_DATA)
  {
    BiasData_t *bias_data = (BiasData_t *)msgdata;
    accel_bias[0] = bias_data->accel_bias[0];
    accel_bias[1] = bias_data->accel_bias[1];
    accel_bias[2] = bias_data->accel_bias[2];
    bias_initialized = bias_data->initialized;
    return;
  }

  // Skip data processing until initialization is complete
  if (!init_complete)
  {
    return;
  }

  // Process filtered IMU data
  if (msgid != MSG_ID_FILTERED)
  {
    wrong_msgid_count++;
    return;
  }

  ImuFilteredData_t *filtered = (ImuFilteredData_t *)msgdata;

  // Apply bias correction
  float corrected_ax = filtered->ax;
  float corrected_ay = filtered->ay;
  float corrected_az = filtered->az;

  if (bias_initialized)
  {
    corrected_ax -= accel_bias[0];
    corrected_ay -= accel_bias[1];
    corrected_az -= accel_bias[2];
  }

  // Prepare gyroscope data (convert rad/s to deg/s)
  FusionVector gyroscope = {
      filtered->gx * 180.0f / (float)M_PI,
      filtered->gy * 180.0f / (float)M_PI,
      filtered->gz * 180.0f / (float)M_PI};
  gyroscope = FusionOffsetUpdate(&fusionOffset, gyroscope);

  // Prepare accelerometer data (convert to g units)
  FusionVector accelerometer = {
      corrected_ax / GRAVITY_AMOUNT,
      corrected_ay / GRAVITY_AMOUNT,
      corrected_az / GRAVITY_AMOUNT};

  // Update AHRS
  FusionAhrsUpdateNoMagnetometer(&fusionAhrs, gyroscope, accelerometer, filtered->dt);

  // Get results
  FusionQuaternion fusionQuat = FusionAhrsGetQuaternion(&fusionAhrs);
  FusionVector linearAccel = FusionAhrsGetLinearAcceleration(&fusionAhrs);
  FusionVector earthAccel = FusionAhrsGetEarthAcceleration(&fusionAhrs);

  // Prepare output data
  static AhrsData_t ahrs_data;
  ahrs_data.timestamp = filtered->timestamp;
  ahrs_data.quaternion[0] = fusionQuat.element.w;
  ahrs_data.quaternion[1] = fusionQuat.element.x;
  ahrs_data.quaternion[2] = fusionQuat.element.y;
  ahrs_data.quaternion[3] = fusionQuat.element.z;

  // Earth frame acceleration (gravity removed, in m/s^2)
  ahrs_data.earth_accel[0] = earthAccel.axis.x * GRAVITY_AMOUNT;
  ahrs_data.earth_accel[1] = earthAccel.axis.y * GRAVITY_AMOUNT;
  ahrs_data.earth_accel[2] = earthAccel.axis.z * GRAVITY_AMOUNT;

  // Sensor frame acceleration (gravity removed, in m/s^2)
  ahrs_data.sensor_accel[0] = linearAccel.axis.x * GRAVITY_AMOUNT;
  ahrs_data.sensor_accel[1] = linearAccel.axis.y * GRAVITY_AMOUNT;
  ahrs_data.sensor_accel[2] = linearAccel.axis.z * GRAVITY_AMOUNT;

  // Corrected gyro (in rad/s)
  ahrs_data.gyro_corrected[0] = gyroscope.axis.x * (float)M_PI / 180.0f;
  ahrs_data.gyro_corrected[1] = gyroscope.axis.y * (float)M_PI / 180.0f;
  ahrs_data.gyro_corrected[2] = gyroscope.axis.z * (float)M_PI / 180.0f;

  ahrs_data.dt = filtered->dt;

  // Send to MainCore (which will forward to SubCore3)
  MP.Send(MSG_ID_AHRS, &ahrs_data, 0); // 0 = MainCore

  // Update loop count
  loop_count++;

  // Send statistics periodically (every 5 seconds)
  uint32_t current_time = millis();
  if (current_time - last_stats_time >= 5000)
  {
    static ProcessingStats_t stats;
    stats.core_id = 2;
    stats.loop_count = loop_count; // Use successful message count
    stats.actual_rate_hz = (loop_count - stats_loop_count_start) / 5.0f;
    stats.dropped_messages = wrong_msgid_count; // Only count wrong message IDs

    MP.Send(MSG_ID_STATS, &stats, 0); // Send to MainCore

    last_stats_time = current_time;
    stats_loop_count_start = loop_count;

    // Reset wrong_msgid counter
    wrong_msgid_count = 0;
  }
}
