#include "test_framework.h"
#include "localizer/gyro_bias_fit.h"
#include "localizer/initial_alignment.h"
#include "localizer/localizer_config.h"
#include <random>

static const float G = LOCALIZER_GRAVITY;
static const float LAT = LOCALIZER_LATITUDE_DEG;
static const float W = LOCALIZER_EARTH_RATE;
static const float DEG = 180.0f / LOCALIZER_PI;

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

/** Attitude whose Up is `up` (sensor frame) with an arbitrary heading. */
static Quat attitudeWithUp(Vec3 up, float headingRad)
{
  up = vec3Normalise(up);
  const Vec3 helper = (fabsf(up.x) < 0.9f) ? vec3(1, 0, 0) : vec3(0, 1, 0);
  const Vec3 n0 = vec3Normalise(vec3Cross(vec3Cross(up, helper), up));
  const Vec3 north = vec3Add(vec3Scale(n0, cosf(headingRad)), vec3Scale(vec3Cross(up, n0), sinf(headingRad)));
  return quatFromEarthAxesInSensorFrame(north, vec3Cross(up, north), up);
}

struct Faces
{
  Vec3 accel[6];
  Vec3 gyro[6];
};

/** Six faces with a few degrees of misalignment, random headings and the real noise level of a 10 s mean. */
static Faces syntheticFaces(Vec3 accelBias, Vec3 gyroBias, unsigned seed, float gyroNoise = 3.0e-4f / 180.0f * LOCALIZER_PI)
{
  std::mt19937 rng(seed);
  std::normal_distribution<float> n(0.0f, 1.0f);
  std::uniform_real_distribution<float> heading(0.0f, 2.0f * LOCALIZER_PI);
  const Vec3 omega = earthRateNwu(LAT, W);
  Faces f;
  for (int k = 0; k < 6; k++)
  {
    const Vec3 up = vec3Add(faceUp(k), vec3(0.05f * n(rng), 0.05f * n(rng), 0.05f * n(rng)));
    const Quat q = attitudeWithUp(up, heading(rng));
    f.accel[k] = vec3Add(quatRotateEarthToSensor(q, vec3(0, 0, G)), vec3Add(accelBias, vec3(2e-4f * n(rng), 2e-4f * n(rng), 2e-4f * n(rng))));
    f.gyro[k] = vec3Add(quatRotateEarthToSensor(q, omega), vec3Add(gyroBias, vec3(gyroNoise * n(rng), gyroNoise * n(rng), gyroNoise * n(rng))));
  }
  return f;
}

TEST(gyro_bias_fit_recovers_bias_from_six_faces)
{
  const Vec3 accelBias = vec3(-0.011f, -0.012f, 0.040f);
  // like the real board: 0.006, -0.005, -0.023 deg/s, i.e. larger than the earth rate
  const Vec3 truth = vec3(1.1e-4f, -0.9e-4f, -4.1e-4f);
  float worst = 0.0f, worstVertical = 0.0f;
  for (unsigned seed = 1; seed <= 20; seed++)
  {
    const Faces f = syntheticFaces(accelBias, truth, seed);
    GyroBiasFit fit;
    for (int k = 0; k < 6; k++) CHECK(fit.add(f.accel[k], f.gyro[k]));
    Vec3 b = vec3Zero();
    float rms = 0.0f, vertical = 0.0f;
    CHECK(fit.solve(accelBias, LAT, W, b, &rms));
    CHECK(fit.estimateVerticalRate(accelBias, vertical));
    const float err = vec3Norm(vec3Sub(b, truth));
    if (err > worst) worst = err;
    const float verr = fabsf(vertical - GyroBiasFit::verticalRate(LAT, W));
    if (verr > worstVertical) worstVertical = verr;
    CHECK_LT(rms, 2.0e-5f);
  }
  METRIC("worst gyro bias error (deg/s)", worst * DEG);
  METRIC("worst vertical rate error (deg/h)", worstVertical * DEG * 3600.0f);
  // noise of the 10 s means is 0.00028 deg/s per axis; the bias must come
  // out well below the horizontal earth rate (0.0034 deg/s)
  CHECK_LT(worst * DEG, 0.0008f);
  CHECK_LT(worstVertical * DEG * 3600.0f, 1.5f);
}

TEST(gyro_bias_fit_is_exact_without_noise)
{
  const Vec3 accelBias = vec3(0.02f, -0.015f, 0.03f);
  const Vec3 truth = vec3(-3.0e-4f, 2.0e-4f, 5.0e-4f);
  const Faces f = syntheticFaces(accelBias, truth, 7, 0.0f);
  GyroBiasFit fit;
  for (int k = 0; k < 6; k++) fit.add(f.accel[k], f.gyro[k]);
  Vec3 b = vec3Zero();
  float rms = 1.0f, vertical = 0.0f;
  CHECK(fit.solve(accelBias, LAT, W, b, &rms));
  // the accel means carry 2e-4 m/s^2 of noise: Up is known to ~0.001 deg
  CHECK_NEAR(b.x, truth.x, 1e-7);
  CHECK_NEAR(b.y, truth.y, 1e-7);
  CHECK_NEAR(b.z, truth.z, 1e-7);
  CHECK_LT(rms, 1e-7f);
  CHECK(fit.estimateVerticalRate(accelBias, vertical));
  CHECK_NEAR(vertical, GyroBiasFit::verticalRate(LAT, W), 1e-7);
}

TEST(gyro_bias_fit_needs_diverse_orientations)
{
  GyroBiasFit fit;
  Vec3 b = vec3Zero();
  CHECK(!fit.solve(vec3Zero(), LAT, W, b)); // nothing
  // +Z, -Z and a tilt about X: nothing ever points along X
  fit.add(vec3(0, 0, G), vec3(0, 0, 1e-4f));
  fit.add(vec3(0, 0, -G), vec3(0, 0, 1e-4f));
  fit.add(vec3(0, -G * 0.5f, G * 0.866f), vec3(0, 0, 1e-4f));
  fit.add(vec3(0, 0, G), vec3(0, 0, 1e-4f));
  CHECK(!fit.solve(vec3Zero(), LAT, W, b));
  float vertical = 0.0f;
  CHECK(!fit.estimateVerticalRate(vec3Zero(), vertical));
  // adding +X and -X makes it solvable
  fit.add(vec3(G, 0, 0), vec3(0, 0, 1e-4f));
  fit.add(vec3(-G, 0, 0), vec3(0, 0, 1e-4f));
  CHECK(fit.solve(vec3Zero(), LAT, W, b));
}

TEST(gyro_bias_fit_pairs_cancel_the_latitude_term)
{
  // With exactly opposite faces the bias is the mean of the two readings
  // whatever latitude is assumed: the vertical earth rate cancels.
  const Vec3 truth = vec3(-1.0e-4f, 2.0e-4f, -4.0e-4f);
  const Faces f = syntheticFaces(vec3Zero(), truth, 3, 0.0f);
  GyroBiasFit fit;
  for (int k = 0; k < 6; k++)
  {
    // exact opposite faces (no misalignment) so that the cancellation is exact
    const Vec3 up = faceUp(k);
    const Quat q = attitudeWithUp(up, 0.3f * k);
    fit.add(quatRotateEarthToSensor(q, vec3(0, 0, G)), vec3Add(quatRotateEarthToSensor(q, earthRateNwu(LAT, W)), truth));
  }
  (void)f;
  Vec3 bTokyo = vec3Zero(), bEquator = vec3Zero();
  CHECK(fit.solve(vec3Zero(), LAT, W, bTokyo));
  CHECK(fit.solve(vec3Zero(), 0.0f, W, bEquator));
  CHECK_NEAR(vec3Norm(vec3Sub(bTokyo, truth)), 0.0f, 1e-8);
  CHECK_NEAR(vec3Norm(vec3Sub(bEquator, truth)), 0.0f, 1e-8);
}

TEST(gyro_bias_fit_matches_device_recordings)
{
  // Mean readings of the real board over 15 s at rest (logs/stop_*.bin and
  // desk_move_30cm.bin, 2026-09-05, 36.6-36.8 C). The level and upside-down
  // pair differs by 0.0051 deg/s on z: twice the vertical earth rate at
  // Tokyo (2 x 0.00244 deg/s) plus noise. None of the recordings has X up,
  // so two X faces are synthesised from the bias found offline; the checks
  // below only concern the components the real data determine.
  const Vec3 accelBias = vec3(-0.0011f, -0.0120f, 0.0404f);
  const float toRad = LOCALIZER_PI / 180.0f;
  GyroBiasFit fit;
  fit.add(vec3(0.063f, 0.059f, 9.837f), vec3Scale(vec3(-0.00636f, -0.00519f, -0.02089f), toRad));   // level
  fit.add(vec3(0.071f, -0.039f, -9.762f), vec3Scale(vec3(-0.00529f, -0.00562f, -0.02600f), toRad));  // upside down
  fit.add(vec3(0.088f, -4.559f, 8.719f), vec3Scale(vec3(-0.00844f, -0.00510f, -0.02271f), toRad));   // 30 deg tilt about X
  fit.add(vec3(-0.063f, 0.012f, 9.839f), vec3Scale(vec3(-0.00325f, -0.00031f, -0.02122f), toRad));   // level, yawed ~180 deg
  const Vec3 offline = vec3Scale(vec3(-0.0048f, -0.0028f, -0.02345f), toRad);
  const Vec3 omega = earthRateNwu(LAT, W);
  for (int k = 2; k <= 3; k++) // synthetic +X / -X faces
  {
    const Quat q = attitudeWithUp(faceUp(k), 1.0f);
    fit.add(vec3Add(quatRotateEarthToSensor(q, vec3(0, 0, G)), accelBias), vec3Add(quatRotateEarthToSensor(q, omega), offline));
  }
  Vec3 b = vec3Zero();
  float rms = 0.0f, vertical = 0.0f;
  CHECK(fit.solve(accelBias, LAT, W, b, &rms));
  CHECK(fit.estimateVerticalRate(accelBias, vertical));
  METRIC("z gyro bias from the device pair (deg/s)", b.z * DEG);
  METRIC("vertical earth rate seen by the device (deg/h)", vertical * DEG * 3600.0f);
  METRIC("rms residual (deg/s)", rms * DEG);
  CHECK_NEAR(b.z * DEG, -0.02345f, 0.0005f);
  CHECK_NEAR(vertical * DEG * 3600.0f, GyroBiasFit::verticalRate(LAT, W) * DEG * 3600.0f, 1.5f);
  CHECK_LT(rms * DEG, 0.001f);
}
