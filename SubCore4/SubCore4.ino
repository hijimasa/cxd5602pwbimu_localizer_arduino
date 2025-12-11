/**
 * @file SubCore4.ino
 * @brief Velocity and Position Integration Core
 *
 * SubCore4: Receives ZUPT data, performs velocity and position
 * integration with exponential decay model, then forwards to SubCore5 (Output)
 */

#if (SUBCORE != 4)
#error "Core selection is wrong!! Must select SubCore4"
#endif

#include <MP.h>
#include "../shared_types.h"

// Motion decay parameters
#define MOTION_THRESHOLD 0.1f
#define MIN_DECAY 0.90f
#define MAX_DECAY 0.999f

// State variables
static float velocity[3] = {0.0f, 0.0f, 0.0f};
static float position[3] = {0.0f, 0.0f, 0.0f};
static float old_acceleration[3] = {0.0f, 0.0f, 0.0f};
static float old_velocity[3] = {0.0f, 0.0f, 0.0f};

// Output counter for SubCore5
static int output_counter = 0;
#define OUTPUT_INTERVAL (MESUREMENT_FREQUENCY / 30)  // 30Hz output

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

  // Process ZUPT data
  if (msgid != MSG_ID_ZUPT) {
    return;
  }

  ZuptData_t *zupt = (ZuptData_t *)msgdata;
  float dt = zupt->dt;

  // Calculate motion magnitude for exponential decay
  float motion_magnitude = zupt->accel_magnitude + zupt->gyro_magnitude * 5.0f;

  // Apply exponential velocity decay when motion is small
  if (motion_magnitude < MOTION_THRESHOLD) {
    float normalized_motion = motion_magnitude / MOTION_THRESHOLD;
    float decay_factor = MIN_DECAY + (MAX_DECAY - MIN_DECAY) * normalized_motion;

    velocity[0] *= decay_factor;
    velocity[1] *= decay_factor;
    velocity[2] *= decay_factor;
  }

  // Velocity integration (trapezoidal rule)
  velocity[0] += (zupt->earth_accel[0] + old_acceleration[0]) / 2.0f * dt;
  velocity[1] += (zupt->earth_accel[1] + old_acceleration[1]) / 2.0f * dt;
  velocity[2] += (zupt->earth_accel[2] + old_acceleration[2]) / 2.0f * dt;

  // Position integration (trapezoidal rule)
  position[0] += (velocity[0] + old_velocity[0]) / 2.0f * dt;
  position[1] += (velocity[1] + old_velocity[1]) / 2.0f * dt;
  position[2] += (velocity[2] + old_velocity[2]) / 2.0f * dt;

  // Apply ZUPT correction if stationary
  if (zupt->is_stationary) {
    velocity[0] = 0.0f;
    velocity[1] = 0.0f;
    velocity[2] = 0.0f;
  }

  // Store for next iteration
  old_acceleration[0] = zupt->earth_accel[0];
  old_acceleration[1] = zupt->earth_accel[1];
  old_acceleration[2] = zupt->earth_accel[2];
  old_velocity[0] = velocity[0];
  old_velocity[1] = velocity[1];
  old_velocity[2] = velocity[2];

  // Send output at reduced rate (30Hz)
  output_counter++;
  if (output_counter >= OUTPUT_INTERVAL) {
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
    pos_data.gyro[0] = zupt->gyro_magnitude;  // simplified - just magnitude
    pos_data.gyro[1] = 0.0f;
    pos_data.gyro[2] = 0.0f;
    pos_data.temp = 0.0f;
    pos_data.is_stationary = zupt->is_stationary;

    MP.Send(MSG_ID_POSITION, &pos_data, SUBCORE_OUTPUT);
    output_counter = 0;
  }
}
