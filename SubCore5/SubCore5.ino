/**
 * @file SubCore5.ino
 * @brief Serial Output Core (DEPRECATED - NO LONGER USED)
 *
 * This SubCore is no longer used. Serial output is now handled
 * by MainCore to avoid Serial resource conflicts between cores.
 *
 * Serial output functionality has been moved to MainCore which
 * receives position data from SubCore4 and outputs it directly.
 *
 * The Spresense multicore system now uses only 4 SubCores:
 *   SubCore1: Gaussian Filter
 *   SubCore2: AHRS (Fusion)
 *   SubCore3: ZUPT & Bias Learning
 *   SubCore4: Position Integration
 *   MainCore: IMU Data Acquisition + Serial Output
 */

// This file is kept for reference but should not be compiled or used

#if (SUBCORE != 5)
#error "Core selection is wrong!! Must select SubCore5"
#endif

void setup()
{
  // Do nothing - this core should not be started
}

void loop()
{
  // Do nothing
}

#if 0 // DISABLED - This code is no longer active

#if (SUBCORE != 5)
#error "Core selection is wrong!! Must select SubCore5"
#endif

#include <MP.h>
#include "shared_types.h"

void setup()
{
  // Initialize MP library
  MP.begin();

  // Initialize Serial
  Serial.begin(115200);

  // Set receive timeout to blocking mode
  MP.RecvTimeout(MP_RECV_BLOCKING);
}

void loop()
{
  int8_t msgid;
  PositionData_t *pos_data;

  // Wait for position data from SubCore4
  int ret = MP.Recv(&msgid, &pos_data);
  if (ret < 0)
  {
    return;
  }

  if (msgid != MSG_ID_POSITION)
  {
    return;
  }

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
}

#endif // DISABLED
