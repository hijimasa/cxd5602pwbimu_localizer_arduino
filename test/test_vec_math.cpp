#include "test_framework.h"
#include "localizer/vec_math.h"
#include <random>

extern "C"
{
#include "Fusion/Fusion.h"
}

static Quat randomQuat(std::mt19937 &rng)
{
  std::normal_distribution<float> n(0.0f, 1.0f);
  Quat q = {n(rng), n(rng), n(rng), n(rng)};
  return quatNormalise(q);
}

TEST(vec_math_rotation_matches_fusion_matrix)
{
  std::mt19937 rng(42);
  for (int i = 0; i < 200; i++)
  {
    const Quat q = randomQuat(rng);
    const Vec3 v = vec3(1.0f, -2.0f, 0.5f);
    const Vec3 r = quatRotateSensorToEarth(q, v);

    FusionQuaternion fq;
    fq.element.w = q.w; fq.element.x = q.x; fq.element.y = q.y; fq.element.z = q.z;
    const FusionMatrix m = FusionQuaternionToMatrix(fq);
    const FusionVector fv = {{v.x, v.y, v.z}};
    const FusionVector fr = FusionMatrixMultiplyVector(m, fv);
    CHECK_NEAR(r.x, fr.axis.x, 1e-5);
    CHECK_NEAR(r.y, fr.axis.y, 1e-5);
    CHECK_NEAR(r.z, fr.axis.z, 1e-5);
  }
}

TEST(vec_math_earth_to_sensor_is_inverse)
{
  std::mt19937 rng(7);
  for (int i = 0; i < 100; i++)
  {
    const Quat q = randomQuat(rng);
    const Vec3 v = vec3(0.3f, 0.7f, -1.1f);
    const Vec3 back = quatRotateEarthToSensor(q, quatRotateSensorToEarth(q, v));
    CHECK_NEAR(back.x, v.x, 1e-5);
    CHECK_NEAR(back.y, v.y, 1e-5);
    CHECK_NEAR(back.z, v.z, 1e-5);
  }
}

TEST(vec_math_quat_from_axes_round_trip)
{
  // For any attitude, expressing the Earth axes in the sensor frame and
  // rebuilding the quaternion must give the same rotation (up to sign).
  std::mt19937 rng(3);
  for (int i = 0; i < 500; i++)
  {
    const Quat q = randomQuat(rng);
    const Vec3 north = quatRotateEarthToSensor(q, vec3(1, 0, 0));
    const Vec3 west = quatRotateEarthToSensor(q, vec3(0, 1, 0));
    const Vec3 up = quatRotateEarthToSensor(q, vec3(0, 0, 1));
    const Quat r = quatFromEarthAxesInSensorFrame(north, west, up);
    const float dot = fabsf(q.w * r.w + q.x * r.x + q.y * r.y + q.z * r.z);
    CHECK_NEAR(dot, 1.0f, 1e-4);

    // and Fusion must agree on the gravity direction in the sensor frame
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    FusionQuaternion fq;
    fq.element.w = r.w; fq.element.x = r.x; fq.element.y = r.y; fq.element.z = r.z;
    FusionAhrsSetQuaternion(&ahrs, fq);
    const FusionVector g = FusionAhrsGetGravity(&ahrs);
    CHECK_NEAR(g.axis.x, up.x, 1e-4);
    CHECK_NEAR(g.axis.y, up.y, 1e-4);
    CHECK_NEAR(g.axis.z, up.z, 1e-4);
  }
}

TEST(vec_math_quat_multiply_composes_rotations)
{
  const Quat a = quatNormalise({0.9f, 0.1f, -0.2f, 0.3f});
  const Quat b = quatNormalise({0.5f, 0.5f, 0.5f, -0.2f});
  const Vec3 v = vec3(1.0f, 2.0f, 3.0f);
  const Vec3 lhs = quatRotateSensorToEarth(quatMultiply(a, b), v);
  const Vec3 rhs = quatRotateSensorToEarth(a, quatRotateSensorToEarth(b, v));
  CHECK_NEAR(lhs.x, rhs.x, 1e-4);
  CHECK_NEAR(lhs.y, rhs.y, 1e-4);
  CHECK_NEAR(lhs.z, rhs.z, 1e-4);
}
