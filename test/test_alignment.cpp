#include "test_framework.h"
#include "localizer/initial_alignment.h"
#include "localizer/localizer_config.h"
#include <random>

static Quat randomQuat(std::mt19937 &rng)
{
  std::normal_distribution<float> n(0.0f, 1.0f);
  Quat q = {n(rng), n(rng), n(rng), n(rng)};
  return quatNormalise(q);
}

static const float G = LOCALIZER_GRAVITY;
static const float LAT = LOCALIZER_LATITUDE_DEG;
static const float W = LOCALIZER_EARTH_RATE;

TEST(alignment_earth_rate_vector)
{
  const Vec3 w = earthRateNwu(LAT, W);
  CHECK_NEAR(w.x, W * cosf(LAT * LOCALIZER_PI / 180.0f), 1e-9);
  CHECK_NEAR(w.y, 0.0f, 1e-12);
  CHECK_NEAR(w.z, W * sinf(LAT * LOCALIZER_PI / 180.0f), 1e-9);
}

TEST(alignment_recovers_attitude_from_ideal_sensors)
{
  std::mt19937 rng(11);
  const Vec3 omegaNwu = earthRateNwu(LAT, W);
  for (int i = 0; i < 300; i++)
  {
    const Quat truth = randomQuat(rng);
    const Vec3 meanAccel = quatRotateEarthToSensor(truth, vec3(0, 0, G));
    const Vec3 meanGyro = quatRotateEarthToSensor(truth, omegaNwu);

    const AlignmentResult r = computeAlignment(meanAccel, meanGyro, G, LAT, W);
    CHECK(r.northValid);

    // gravity direction (tilt) and north direction (heading) must match
    const Vec3 upEst = quatRotateEarthToSensor(r.quaternion, vec3(0, 0, 1));
    const Vec3 upTrue = vec3Normalise(meanAccel);
    CHECK_LT(vec3AngleBetween(upEst, upTrue) * 180.0f / LOCALIZER_PI, 0.01f);

    const Vec3 northEst = quatRotateEarthToSensor(r.quaternion, vec3(1, 0, 0));
    const Vec3 northTrue = quatRotateEarthToSensor(truth, vec3(1, 0, 0));
    CHECK_LT(vec3AngleBetween(northEst, northTrue) * 180.0f / LOCALIZER_PI, 0.05f);

    // no gyro offset and no accel bias for ideal sensors
    CHECK_NEAR(vec3Norm(r.gyroOffset), 0.0f, 1e-8);
    CHECK_NEAR(vec3Norm(r.accelBias), 0.0f, 1e-4);
  }
}

TEST(alignment_tilted_start_regression)
{
  // Regression for the original sign error: a 30 deg roll must not produce
  // a quaternion whose gravity direction is 60 deg off.
  const float roll = 30.0f * LOCALIZER_PI / 180.0f;
  const Quat truth = {cosf(roll / 2), sinf(roll / 2), 0.0f, 0.0f};
  const Vec3 meanAccel = quatRotateEarthToSensor(truth, vec3(0, 0, G));
  const Vec3 meanGyro = quatRotateEarthToSensor(truth, earthRateNwu(LAT, W));
  const AlignmentResult r = computeAlignment(meanAccel, meanGyro, G, LAT, W);
  const float dot = fabsf(truth.w * r.quaternion.w + truth.x * r.quaternion.x + truth.y * r.quaternion.y + truth.z * r.quaternion.z);
  CHECK_NEAR(dot, 1.0f, 1e-5);
}

TEST(alignment_gyro_offset_and_accel_bias_are_recovered)
{
  const Quat truth = quatNormalise({0.8f, 0.3f, -0.2f, 0.4f});
  const Vec3 gyroBias = vec3(1e-3f, -2e-3f, 5e-4f);
  const Vec3 meanGyro = vec3Add(quatRotateEarthToSensor(truth, earthRateNwu(LAT, W)), gyroBias);
  // bias purely along gravity is observable
  const Vec3 up = quatRotateEarthToSensor(truth, vec3(0, 0, 1));
  const Vec3 meanAccel = vec3Scale(up, G + 0.05f);

  const AlignmentResult r = computeAlignment(meanAccel, meanGyro, G, LAT, W);
  // heading is still defined by earth rate + bias here, so only check the
  // residual identity: meanGyro - offset - earthRateSensor == 0
  const Vec3 residual = vec3Sub(vec3Sub(meanGyro, r.gyroOffset), r.earthRateSensor);
  CHECK_NEAR(vec3Norm(residual), 0.0f, 1e-8);
  CHECK_NEAR(vec3Norm(r.accelBias), 0.05f, 1e-4);
  CHECK_NEAR(vec3Dot(vec3Normalise(r.accelBias), up), 1.0f, 1e-4);
}

TEST(alignment_handles_zero_horizontal_rate)
{
  // mean gyro exactly parallel to gravity: North is undefined but the result
  // must still be a valid unit quaternion with the right gravity direction.
  const Vec3 meanAccel = vec3(0.0f, 0.0f, G);
  const Vec3 meanGyro = vec3(0.0f, 0.0f, 1e-4f);
  const AlignmentResult r = computeAlignment(meanAccel, meanGyro, G, LAT, W);
  CHECK(!r.northValid);
  const float n = sqrtf(r.quaternion.w * r.quaternion.w + r.quaternion.x * r.quaternion.x + r.quaternion.y * r.quaternion.y + r.quaternion.z * r.quaternion.z);
  CHECK_NEAR(n, 1.0f, 1e-5);
  CHECK(std::isfinite(r.gyroOffset.x) && std::isfinite(r.gyroOffset.y) && std::isfinite(r.gyroOffset.z));
  const Vec3 upEst = quatRotateEarthToSensor(r.quaternion, vec3(0, 0, 1));
  CHECK_NEAR(upEst.z, 1.0f, 1e-5);
}

TEST(alignment_accumulator_averages)
{
  InitialAlignment acc(4);
  CHECK(!acc.add(vec3(1, 0, 0), vec3(0, 1, 0)));
  CHECK(!acc.add(vec3(3, 0, 0), vec3(0, 3, 0)));
  CHECK(!acc.add(vec3(1, 2, 0), vec3(0, 1, 2)));
  CHECK(acc.add(vec3(3, 2, 0), vec3(0, 3, 2)));
  CHECK(acc.isComplete());
  CHECK_NEAR(acc.meanAccel().x, 2.0f, 1e-6);
  CHECK_NEAR(acc.meanAccel().y, 1.0f, 1e-6);
  CHECK_NEAR(acc.meanGyro().y, 2.0f, 1e-6);
  CHECK_NEAR(acc.meanGyro().z, 1.0f, 1e-6);
  // extra samples after completion are ignored
  CHECK(acc.add(vec3(100, 100, 100), vec3(100, 100, 100)));
  CHECK_NEAR(acc.meanAccel().x, 2.0f, 1e-6);
}
