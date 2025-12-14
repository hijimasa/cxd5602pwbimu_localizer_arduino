/**
 * @file SubCore4.ino
 * @brief Velocity and Position Integration Core
 *
 * SubCore4: Receives ZUPT data, performs velocity and position
 * integration with exponential decay model, then sends to MainCore for output
 */

#if (SUBCORE != 4)
#error "Core selection is wrong!! Must select SubCore4"
#endif

#include <MP.h>
#include "shared_types.h"

// State variables
static float velocity[3] = {0.0f, 0.0f, 0.0f};
static float position[3] = {0.0f, 0.0f, 0.0f};
static float old_acceleration[3] = {0.0f, 0.0f, 0.0f};
static float old_velocity[3] = {0.0f, 0.0f, 0.0f};
static bool integration_initialized = false;

// Output counter for MainCore
static int output_counter = 0;
#define OUTPUT_INTERVAL (MESUREMENT_FREQUENCY / 30) // 30Hz output

// Statistics tracking
static uint32_t loop_count = 0;
static uint32_t last_stats_time = 0;
static uint32_t stats_loop_count_start = 0;
static uint32_t recv_error_count = 0;
static uint32_t wrong_msgid_count = 0;

void setup()
{
  // Initialize MP library
  MP.begin();

  // Set receive timeout to blocking mode
  MP.RecvTimeout(MP_RECV_BLOCKING);

  last_stats_time = millis();

  // Send startup notification to MainCore
  static ProcessingStats_t startup_stats;
  startup_stats.core_id = 4;
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

  // Process ZUPT data
  if (msgid != MSG_ID_ZUPT)
  {
    wrong_msgid_count++;
    return;
  }

  ZuptData_t *zupt = (ZuptData_t *)msgdata;
  float dt = zupt->dt;

  // Initialize integration with first data to avoid jump
  if (!integration_initialized)
  {
    old_acceleration[0] = zupt->earth_accel[0];
    old_acceleration[1] = zupt->earth_accel[1];
    old_acceleration[2] = zupt->earth_accel[2];
    integration_initialized = true;
  }

  // Calculate gyro magnitude from filtered (not bias-corrected) gyro
  // This matches the old implementation which used raw filtered gyro for motion detection
  float gyro_magnitude_filtered = sqrtf(
      zupt->gyro_filtered[0] * zupt->gyro_filtered[0] +
      zupt->gyro_filtered[1] * zupt->gyro_filtered[1] +
      zupt->gyro_filtered[2] * zupt->gyro_filtered[2]);

  // Apply dead zone to small accelerations to prevent noise integration
  float accel_x = zupt->earth_accel[0];
  float accel_y = zupt->earth_accel[1];
  float accel_z = zupt->earth_accel[2];

  // Velocity integration (trapezoidal rule)
  velocity[0] += (accel_x + old_acceleration[0]) / 2.0f * dt;
  velocity[1] += (accel_y + old_acceleration[1]) / 2.0f * dt;
  velocity[2] += (accel_z + old_acceleration[2]) / 2.0f * dt;

  // Position integration (trapezoidal rule)
  position[0] += (velocity[0] + old_velocity[0]) / 2.0f * dt;
  position[1] += (velocity[1] + old_velocity[1]) / 2.0f * dt;
  position[2] += (velocity[2] + old_velocity[2]) / 2.0f * dt;

  // Apply ZUPT correction if stationary
  if (zupt->is_stationary)
  {
    velocity[0] = 0.0f;
    velocity[1] = 0.0f;
    velocity[2] = 0.0f;

    // Bias update is disabled for now to diagnose drift issues
    // The initial bias from MainCore should be sufficient
    // Continuous bias update may cause oscillation or over-correction
    // TODO: Re-enable with proper convergence detection
    (void)zupt->linear_accel_raw; // Suppress unused warning
  }
  else
  {
    // Apply velocity magnitude limit to prevent runaway integration
    // This helps contain accumulated errors at high sampling rates
    float vel_magnitude = sqrtf(velocity[0] * velocity[0] +
                                velocity[1] * velocity[1] +
                                velocity[2] * velocity[2]);
    const float MAX_VELOCITY = 5.0f; // m/s (reasonable human walking/running speed)
    if (vel_magnitude > MAX_VELOCITY)
    {
      float scale = MAX_VELOCITY / vel_magnitude;
      velocity[0] *= scale;
      velocity[1] *= scale;
      velocity[2] *= scale;
    }
  }

  // Store for next iteration (with dead zone applied)
  old_acceleration[0] = accel_x;
  old_acceleration[1] = accel_y;
  old_acceleration[2] = accel_z;
  old_velocity[0] = velocity[0];
  old_velocity[1] = velocity[1];
  old_velocity[2] = velocity[2];

  // Update loop count
  loop_count++;

  // Send output at reduced rate (30Hz) to MainCore
  output_counter++;
  if (output_counter >= OUTPUT_INTERVAL)
  {
    static PositionData_t pos_data;
    pos_data.timestamp = zupt->timestamp;
    pos_data.quaternion[0] = zupt->quaternion[0];
    pos_data.quaternion[1] = zupt->quaternion[1];
    pos_data.quaternion[2] = zupt->quaternion[2];
    pos_data.quaternion[3] = zupt->quaternion[3];
    pos_data.velocity[0] = velocity[0];
    pos_data.velocity[1] = velocity[1];
    pos_data.velocity[2] = velocity[2];
    pos_data.position[0] = position[0];
    pos_data.position[1] = position[1];
    pos_data.position[2] = position[2];
    pos_data.acceleration[0] = zupt->earth_accel[0];
    pos_data.acceleration[1] = zupt->earth_accel[1];
    pos_data.acceleration[2] = zupt->earth_accel[2];
    pos_data.gyro[0] = zupt->gyro_corrected[0];
    pos_data.gyro[1] = zupt->gyro_corrected[1];
    pos_data.gyro[2] = zupt->gyro_corrected[2];
    pos_data.temp = zupt->temp;
    pos_data.is_stationary = zupt->is_stationary;

    // Send position data to MainCore (not SubCore5)
    MP.Send(MSG_ID_POSITION, &pos_data, 0); // 0 = MainCore
    output_counter = 0;
  }

  // Send statistics every 5 seconds
  uint32_t current_time = millis();
  if (current_time - last_stats_time >= 5000)
  {
    static ProcessingStats_t stats;
    stats.core_id = 4;
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
