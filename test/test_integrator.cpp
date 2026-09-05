#include "test_framework.h"
#include "localizer/velocity_integrator.h"
#include <random>

// ---- Reference: original SubCore4 implementation (verbatim port) ----------
namespace ref
{
static float velocity[3], position[3], old_acceleration[3], old_velocity[3];
static bool integration_initialized;
static void reset()
{
  for (int i = 0; i < 3; i++) velocity[i] = position[i] = old_acceleration[i] = old_velocity[i] = 0.0f;
  integration_initialized = false;
}
static void update(const float *earth_accel, float dt, bool is_stationary)
{
  if (!integration_initialized)
  {
    for (int i = 0; i < 3; i++) old_acceleration[i] = earth_accel[i];
    integration_initialized = true;
  }
  float accel_x = earth_accel[0], accel_y = earth_accel[1], accel_z = earth_accel[2];
  velocity[0] += (accel_x + old_acceleration[0]) / 2.0f * dt;
  velocity[1] += (accel_y + old_acceleration[1]) / 2.0f * dt;
  velocity[2] += (accel_z + old_acceleration[2]) / 2.0f * dt;
  position[0] += (velocity[0] + old_velocity[0]) / 2.0f * dt;
  position[1] += (velocity[1] + old_velocity[1]) / 2.0f * dt;
  position[2] += (velocity[2] + old_velocity[2]) / 2.0f * dt;
  if (is_stationary)
  {
    velocity[0] = velocity[1] = velocity[2] = 0.0f;
  }
  else
  {
    float vel_magnitude = sqrtf(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2]);
    const float MAX_VELOCITY = LOCALIZER_MAX_VELOCITY;
    if (vel_magnitude > MAX_VELOCITY)
    {
      float scale = MAX_VELOCITY / vel_magnitude;
      velocity[0] *= scale; velocity[1] *= scale; velocity[2] *= scale;
    }
  }
  old_acceleration[0] = accel_x; old_acceleration[1] = accel_y; old_acceleration[2] = accel_z;
  old_velocity[0] = velocity[0]; old_velocity[1] = velocity[1]; old_velocity[2] = velocity[2];
}
} // namespace ref

TEST(integrator_matches_reference_implementation)
{
  std::mt19937 rng(9);
  std::normal_distribution<float> n(0.0f, 1.0f);
  VelocityIntegrator integ;
  ref::reset();
  const float dt = LOCALIZER_NOMINAL_DT;
  for (int i = 0; i < 30000; i++)
  {
    const float a[3] = {n(rng) * 3.0f, n(rng) * 3.0f, n(rng) * 0.5f};
    const bool stationary = ((i / 2000) % 3) == 2;
    ref::update(a, dt, stationary);
    integ.update(vec3(a[0], a[1], a[2]), dt, stationary);
    CHECK_NEAR(integ.velocity().x, ref::velocity[0], 1e-6);
    CHECK_NEAR(integ.velocity().y, ref::velocity[1], 1e-6);
    CHECK_NEAR(integ.velocity().z, ref::velocity[2], 1e-6);
    CHECK_NEAR(integ.position().x, ref::position[0], 1e-5);
    CHECK_NEAR(integ.position().y, ref::position[1], 1e-5);
    CHECK_NEAR(integ.position().z, ref::position[2], 1e-5);
    if (testFailures() > 0) break;
  }
}

TEST(integrator_constant_acceleration)
{
  VelocityIntegrator integ;
  const float dt = 0.001f;
  const int N = 2000; // 2 s
  for (int i = 0; i < N; i++) integ.update(vec3(1.0f, 0.0f, 0.0f), dt, false);
  const float T = N * dt;
  CHECK_NEAR(integ.velocity().x, T, 1e-3);
  CHECK_NEAR(integ.position().x, 0.5f * T * T, 2e-3);
}

TEST(integrator_zupt_zeroes_velocity_and_clamps_speed)
{
  VelocityIntegrator integ;
  for (int i = 0; i < 5000; i++) integ.update(vec3(20.0f, 0.0f, 0.0f), 0.001f, false);
  CHECK_NEAR(vec3Norm(integ.velocity()), LOCALIZER_MAX_VELOCITY, 1e-3);
  integ.update(vec3Zero(), 0.001f, true);
  CHECK_NEAR(vec3Norm(integ.velocity()), 0.0f, 1e-9);
}
