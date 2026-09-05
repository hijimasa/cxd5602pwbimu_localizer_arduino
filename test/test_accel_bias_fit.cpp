#include "test_framework.h"
#include "localizer/accel_bias_fit.h"
#include "localizer/localizer_config.h"
#include "imu_simulator.h"
#include <random>

static const float G = LOCALIZER_GRAVITY;

static Vec3 faceUp(int k)
{
  switch (k)
  {
  case 0: return vec3(0, 0, 1);
  case 1: return vec3(0, 0, -1);
  case 2: return vec3(1, 0, 0);
  case 3: return vec3(-1, 0, 0);
  case 4: return vec3(0, 1, 0);
  default: return vec3(0, -1, 0);
  }
}

TEST(accel_bias_fit_recovers_bias_from_six_faces)
{
  std::mt19937 rng(21);
  std::normal_distribution<float> n(0.0f, 1.0f);
  const Vec3 truth = vec3(-0.011f, -0.012f, 0.040f);
  AccelBiasFit fit;
  for (int k = 0; k < 6; k++)
  {
    // faces are never placed exactly: add up to ~5 deg of misalignment
    Vec3 up = vec3Add(faceUp(k), vec3(0.05f * n(rng), 0.05f * n(rng), 0.05f * n(rng)));
    up = vec3Normalise(up);
    const Vec3 mean = vec3Add(vec3Scale(up, G), vec3Add(truth, vec3(2e-4f * n(rng), 2e-4f * n(rng), 2e-4f * n(rng))));
    CHECK(fit.add(mean));
  }
  Vec3 b = vec3Zero();
  float rms = 0.0f;
  CHECK(fit.solve(G, b, &rms));
  CHECK_NEAR(b.x, truth.x, 1e-3);
  CHECK_NEAR(b.y, truth.y, 1e-3);
  CHECK_NEAR(b.z, truth.z, 1e-3);
  CHECK_LT(rms, 1e-3f);
}

TEST(accel_bias_fit_needs_diverse_orientations)
{
  AccelBiasFit fit;
  Vec3 b = vec3Zero();
  CHECK(!fit.solve(G, b)); // nothing
  fit.add(vec3(0, 0, G + 0.04f));
  fit.add(vec3(0, 0, G + 0.04f));
  fit.add(vec3(0.001f, 0, G + 0.04f));
  CHECK(!fit.solve(G, b)); // all the same face: singular
  // two faces plus one tilted: solvable
  AccelBiasFit fit2;
  fit2.add(vec3(0, 0, G + 0.04f));
  fit2.add(vec3(0, 0, -G + 0.04f));
  fit2.add(vec3(G * 0.7071f, 0, G * 0.7071f + 0.04f));
  fit2.add(vec3(0, G * 0.7071f - 0.02f, G * 0.7071f + 0.04f));
  CHECK(fit2.solve(G, b));
  CHECK_NEAR(b.z, 0.04f, 2e-3);
  CHECK_NEAR(b.y, -0.02f, 5e-3);
}

TEST(accel_bias_fit_matches_device_recordings)
{
  // Means measured on the real board (five stationary orientations, see the
  // logs analysis); the fit must reproduce the values found offline.
  AccelBiasFit fit;
  fit.add(vec3(0.0631f, 0.0591f, 9.8373f));  // level
  fit.add(vec3(0.0711f, -0.0388f, -9.7620f)); // upside down
  fit.add(vec3(0.0878f, -4.5585f, 8.7186f));  // 30 deg tilt
  fit.add(vec3(9.7822f, -0.4751f, 0.2938f));  // rolled 90 deg (hand held)
  fit.add(vec3(-0.0653f, 0.0202f, 9.8428f));  // level, another day-time
  Vec3 b = vec3Zero();
  float rms = 0.0f;
  CHECK(fit.solve(G, b, &rms));
  CHECK_NEAR(b.x, -0.0011f, 3e-3);
  CHECK_NEAR(b.y, -0.0120f, 3e-3);
  CHECK_NEAR(b.z, 0.0404f, 3e-3);
  CHECK_LT(rms, 0.01f);
}

TEST(localizer_accel_bias_preset_removes_unobservable_components)
{
  // A sensor bias with horizontal components is calibrated at level, then
  // the sensor rests rolled by 90 deg (where the former horizontal bias
  // becomes vertical/horizontal in the other axes). Without the preset the
  // residual acceleration at the rolled rest is the unobserved bias.
  SimConfig cfg;
  cfg.accelBias = vec3(0.02f, -0.015f, 0.03f);
  const Quat rolled = quatFromAxisAngle(vec3(1, 0, 0), 90.0f / (180.0f / LOCALIZER_PI));
  TrajectoryFn traj = rotationTrajectory({1, 0, 0, 0}, 16.0, 1.0, vec3(1, 0, 0), 90.0f / (180.0f / LOCALIZER_PI));
  (void)rolled;
  float residual[2];
  for (int variant = 0; variant < 2; variant++)
  {
    Localizer loc;
    loc.setBiasLearning(false); // isolate the preset effect
    if (variant == 1) loc.setAccelBiasPreset(cfg.accelBias);
    ImuSimulator sim(cfg);
    float maxRes = 0.0f;
    for (uint64_t k = 0; k < (uint64_t)(22.0 * LOCALIZER_SAMPLE_RATE); k++)
    {
      const double t = (double)k / LOCALIZER_SAMPLE_RATE;
      if (!loc.update(sim.sample(k, traj))) continue;
      if (t > 20.0)
      {
        const float r = vec3Norm(loc.state().earthAccel);
        if (r > maxRes) maxRes = r;
      }
    }
    residual[variant] = maxRes;
  }
  METRIC("residual accel at rolled rest without preset (m/s^2)", residual[0]);
  METRIC("residual accel at rolled rest with preset (m/s^2)", residual[1]);
  CHECK_LT(residual[1] * 3.0f, residual[0]);
  CHECK_LT(residual[1], 0.005f);
}
