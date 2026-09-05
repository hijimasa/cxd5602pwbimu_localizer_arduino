#include "initial_alignment.h"
#include "localizer_config.h"

Vec3 earthRateNwu(float latitudeDeg, float earthRate)
{
  const float lat = latitudeDeg * LOCALIZER_PI / 180.0f;
  return vec3(earthRate * cosf(lat), 0.0f, earthRate * sinf(lat));
}

AlignmentResult computeAlignment(Vec3 meanAccel, Vec3 meanGyro,
                                 float gravity, float latitudeDeg, float earthRate)
{
  AlignmentResult r;
  r.meanAccel = meanAccel;
  r.meanGyro = meanGyro;

  // Up axis: direction of the measured specific force at rest.
  const Vec3 up = vec3Normalise(meanAccel);

  // Horizontal component of the mean angular rate points North.
  Vec3 horizontal = vec3Sub(meanGyro, vec3Scale(up, vec3Dot(meanGyro, up)));
  const float horizontalNorm = vec3Norm(horizontal);
  r.northValid = horizontalNorm > 1.0e-9f;

  Vec3 north;
  if (r.northValid)
  {
    north = vec3Scale(horizontal, 1.0f / horizontalNorm);
  }
  else
  {
    // No usable horizontal rate: pick any axis perpendicular to Up.
    Vec3 helper = (fabsf(up.x) < 0.9f) ? vec3(1.0f, 0.0f, 0.0f) : vec3(0.0f, 1.0f, 0.0f);
    north = vec3Normalise(vec3Cross(vec3Cross(up, helper), up));
  }
  const Vec3 west = vec3Cross(up, north);

  r.quaternion = quatFromEarthAxesInSensorFrame(north, west, up);

  // Expected earth rotation in the sensor frame for this attitude.
  const Vec3 omegaNwu = earthRateNwu(latitudeDeg, earthRate);
  r.earthRateSensor = vec3Add(vec3Scale(north, omegaNwu.x), vec3Scale(up, omegaNwu.z));

  // Whatever remains after removing the earth rotation is gyro offset.
  r.gyroOffset = vec3Sub(meanGyro, r.earthRateSensor);

  // Only the magnitude discrepancy along gravity is observable at a single
  // orientation; components perpendicular to gravity are absorbed by the tilt.
  r.accelBias = vec3Sub(meanAccel, vec3Scale(up, gravity));
  return r;
}

InitialAlignment::InitialAlignment(uint32_t requiredSamples)
    : required_(requiredSamples)
{
  reset();
}

void InitialAlignment::reset()
{
  count_ = 0;
  for (int i = 0; i < 3; i++)
  {
    accelSum_[i] = 0.0;
    gyroSum_[i] = 0.0;
  }
}

bool InitialAlignment::add(Vec3 accel, Vec3 gyro)
{
  if (count_ >= required_)
  {
    return true;
  }
  accelSum_[0] += accel.x;
  accelSum_[1] += accel.y;
  accelSum_[2] += accel.z;
  gyroSum_[0] += gyro.x;
  gyroSum_[1] += gyro.y;
  gyroSum_[2] += gyro.z;
  count_++;
  return count_ >= required_;
}

Vec3 InitialAlignment::meanAccel() const
{
  if (count_ == 0) return vec3Zero();
  return vec3((float)(accelSum_[0] / count_), (float)(accelSum_[1] / count_), (float)(accelSum_[2] / count_));
}

Vec3 InitialAlignment::meanGyro() const
{
  if (count_ == 0) return vec3Zero();
  return vec3((float)(gyroSum_[0] / count_), (float)(gyroSum_[1] / count_), (float)(gyroSum_[2] / count_));
}

AlignmentResult InitialAlignment::compute(float gravity, float latitudeDeg, float earthRate) const
{
  return computeAlignment(meanAccel(), meanGyro(), gravity, latitudeDeg, earthRate);
}
