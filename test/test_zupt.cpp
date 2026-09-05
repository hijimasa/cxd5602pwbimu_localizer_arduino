#include "test_framework.h"
#include "localizer/zupt_detector.h"
#include <random>

static const int SETTLE = ZUPT_WINDOW_SIZE + ZUPT_REQUIRED_SAMPLES + 5;

static bool feedConstant(ZuptDetector &det, Vec3 a, Vec3 g, int n = SETTLE)
{
  bool s = false;
  for (int i = 0; i < n; i++) s = det.update(a, g);
  return s;
}

TEST(zupt_declares_stationary_after_confirmation_period)
{
  ZuptDetector det;
  int firstStationary = -1;
  for (int i = 0; i < 400; i++)
  {
    if (det.update(vec3(0.05f, 0.05f, 0.05f), vec3(0.001f, 0.001f, 0.001f)) && firstStationary < 0)
    {
      firstStationary = i;
    }
  }
  // the first statistics are computed when the window fills (index 15), so
  // the counter strictly exceeds REQUIRED_SAMPLES at index 15 + REQUIRED_SAMPLES
  CHECK(firstStationary == ZUPT_WINDOW_SIZE - 1 + ZUPT_REQUIRED_SAMPLES);
}

TEST(zupt_noisy_rest_is_stationary)
{
  std::mt19937 rng(5);
  std::normal_distribution<float> n(0.0f, 1.0f);
  ZuptDetector det;
  bool s = false;
  int stationaryCount = 0;
  for (int i = 0; i < 4000; i++)
  {
    // residual bias 0.05 m/s^2 plus filtered-noise level scatter
    const Vec3 a = vec3(0.03f + n(rng) * 0.005f, -0.04f + n(rng) * 0.005f, n(rng) * 0.005f);
    const Vec3 g = vec3(n(rng) * 0.0005f, 0.002f + n(rng) * 0.0005f, n(rng) * 0.0005f);
    s = det.update(a, g);
    if (s) stationaryCount++;
  }
  CHECK(s);
  CHECK_LT(4000 - SETTLE, stationaryCount + 1);
}

TEST(zupt_exits_immediately_on_vibration)
{
  ZuptDetector det;
  CHECK(feedConstant(det, vec3Zero(), vec3Zero()));
  CHECK(!det.update(vec3(5.0f, 0.0f, 0.0f), vec3Zero()));
  CHECK(!det.isStationary());
}

TEST(zupt_constant_acceleration_is_motion)
{
  // Locally constant acceleration (extremum of a smooth move) has zero
  // variance but must not be treated as rest.
  ZuptDetector det;
  CHECK(!feedConstant(det, vec3(1.0f, 0.0f, 0.0f), vec3Zero()));
  CHECK(det.isLowVariance());
  CHECK(!det.isStationary());
  // just below the mean threshold it is rest
  ZuptDetector det2;
  CHECK(feedConstant(det2, vec3(0.9f * ZUPT_ACCEL_MEAN_THRESHOLD, 0.0f, 0.0f), vec3Zero()));
  // just above it is motion
  ZuptDetector det3;
  CHECK(!feedConstant(det3, vec3(1.1f * ZUPT_ACCEL_MEAN_THRESHOLD, 0.0f, 0.0f), vec3Zero()));
}

TEST(zupt_constant_rotation_is_motion)
{
  ZuptDetector det;
  CHECK(!feedConstant(det, vec3Zero(), vec3(0.0f, 0.0f, 1.5f * ZUPT_GYRO_MEAN_THRESHOLD)));
  CHECK(det.isLowVariance());
  ZuptDetector det2;
  CHECK(feedConstant(det2, vec3Zero(), vec3(0.0f, 0.0f, 0.5f * ZUPT_GYRO_MEAN_THRESHOLD)));
}

TEST(zupt_mean_threshold_uses_vector_norm)
{
  // three axes each below the threshold but the norm above it -> motion
  ZuptDetector det;
  const float c = 0.7f * ZUPT_ACCEL_MEAN_THRESHOLD; // norm = 1.21 * threshold
  CHECK(!feedConstant(det, vec3(c, c, c), vec3Zero()));
}

TEST(zupt_low_variance_flag_tracks_variance_only)
{
  ZuptDetector det;
  feedConstant(det, vec3(2.0f, 0.0f, 0.0f), vec3Zero());
  CHECK(det.isLowVariance());
  CHECK(!det.isStationary());
  det.update(vec3(2.0f, 3.0f, 0.0f), vec3Zero());
  CHECK(!det.isLowVariance());
}

TEST(zupt_reset_clears_state)
{
  ZuptDetector det;
  CHECK(feedConstant(det, vec3Zero(), vec3Zero()));
  det.reset();
  CHECK(!det.isStationary());
  CHECK(!det.isLowVariance());
  CHECK(!det.update(vec3Zero(), vec3Zero()));
}
