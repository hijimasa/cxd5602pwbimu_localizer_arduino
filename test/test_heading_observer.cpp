/**
 * Unit tests of the heading observer (gyro-compass re-estimation at rest).
 * The gyro means are synthesised from a true attitude, the earth rotation
 * and a bias; the AHRS attitude handed to the observer is the true one
 * turned about Up by a yaw error, which the observer has to find.
 */
#include "test_framework.h"
#include "imu_simulator.h"
#include "localizer/heading_observer.h"
#include "localizer/gyro_bias_fit.h"

static const float DEG = 180.0f / LOCALIZER_PI;
static const float DEG_PER_S = LOCALIZER_PI / 180.0f;

static Vec3 omegaNwu() { return earthRateNwu(LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE); }

/** Gyro mean at rest for a true attitude and a bias. */
static Vec3 restMean(Quat trueAttitude, Vec3 bias)
{
  return vec3Add(quatRotateEarthToSensor(trueAttitude, omegaNwu()), bias);
}

/** AHRS attitude whose frame is wrong by yawError about Up: q_true = Rz(yawError) q_ahrs. */
static Quat ahrsWithYawError(Quat trueAttitude, float yawErrorDeg)
{
  return quatNormalise(quatMultiply(quatAboutUp(-yawErrorDeg / DEG), trueAttitude));
}

static float wrapDeg(float a)
{
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static Quat tiltedAttitude(float yawDeg)
{
  const Quat roll = quatFromAxisAngle(vec3(1, 0, 0), 25.0f / DEG);
  const Quat pitch = quatFromAxisAngle(vec3(0, 1, 0), -15.0f / DEG);
  return quatMultiply(quatAboutUp(yawDeg / DEG), quatMultiply(pitch, roll));
}

static const Vec3 BIAS = {1.1e-4f, -0.9e-4f, -4.1e-4f}; // rad/s, the real board

TEST(heading_observer_single_period_reproduces_gyro_compass)
{
  // One period, exact preset: the yaw error must come out exactly, the
  // bias must stay at the preset, and applying the correction must leave
  // nothing pending.
  const Quat truth = tiltedAttitude(40.0f);
  const Quat ahrs = ahrsWithYawError(truth, 20.0f);
  HeadingObserver obs;
  obs.reset(BIAS, LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE);
  obs.addPeriod(restMean(truth, BIAS), 15.0f, ahrs);
  CHECK(obs.solve(ahrs));
  METRIC("yaw error found (deg, truth 20)", obs.pendingYaw() * DEG);
  METRIC("bias error (deg/s)", vec3Norm(vec3Sub(obs.bias(), BIAS)) * DEG);
  CHECK_NEAR(obs.pendingYaw() * DEG, 20.0f, 0.01f);
  CHECK_LT(vec3Norm(vec3Sub(obs.bias(), BIAS)) * DEG, 1.0e-5f);
  CHECK_NEAR(obs.status().yawScale, 1.0f, 1.0e-3f);
  CHECK_NEAR(obs.status().rateMagnitude, LOCALIZER_EARTH_RATE, 1.0e-8f);
  CHECK_NEAR(obs.status().verticalRate, GyroBiasFit::verticalRate(LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE), 1.0e-8f);

  obs.applyYaw(obs.pendingYaw());
  CHECK(obs.solve(ahrs));
  CHECK_LT(fabsf(obs.pendingYaw()) * DEG, 0.01f);
  CHECK_NEAR(obs.status().yawApplied * DEG, 20.0f, 0.01f);
}

TEST(heading_observer_east_west_separates_a_y_bias_from_the_earth_rate)
{
  // The user's symptom: with the y-axis preset off by more than the
  // horizontal earth rate, x East and x West both read "x East". A second
  // rest at the opposite heading makes the bias observable.
  const Vec3 presetError = vec3(0.0f, 0.006f * DEG_PER_S, 0.0f);
  const Vec3 preset = vec3Add(BIAS, presetError);
  const float yawError = 150.0f; // consistent frame error of the AHRS during both rests
  const Quat east = quatAboutUp(-90.0f / DEG); // sensor x East (level)
  const Quat west = quatAboutUp(90.0f / DEG);

  HeadingObserver obs;
  obs.reset(preset, LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE);
  obs.addPeriod(restMean(east, BIAS), 15.0f, ahrsWithYawError(east, yawError));
  CHECK(obs.solve(ahrsWithYawError(east, yawError)));
  const float singleErr = wrapDeg(obs.pendingYaw() * DEG - yawError);
  METRIC("heading error after one rest (deg)", singleErr);
  CHECK_LT(90.0f, fabsf(singleErr)); // the symptom: the mirror image heading

  obs.addPeriod(restMean(west, BIAS), 10.0f, ahrsWithYawError(west, yawError));
  CHECK(obs.solve(ahrsWithYawError(west, yawError)));
  const float twoErr = wrapDeg(obs.pendingYaw() * DEG - yawError);
  const float biasErr = vec3Norm(vec3Sub(obs.bias(), BIAS)) * DEG;
  METRIC("heading error after East and West rests (deg)", twoErr);
  METRIC("bias error after East and West rests (deg/s)", biasErr);
  METRIC("preset error (deg/s)", vec3Norm(presetError) * DEG);
  // what remains is the pull of the prior (weight 1/16 of a 3 s mean)
  CHECK_LT(fabsf(twoErr), 1.0f);
  CHECK_LT(biasErr, 0.0001f);
  CHECK_LT(obs.status().residualRms * DEG, 0.0001f);
}

TEST(heading_observer_vertical_bias_is_observable_at_one_heading)
{
  // The Up component of the earth rate is known, so a z-axis preset error
  // is corrected from a single rest; the horizontal components stay at the
  // preset (bias and heading cannot be told apart at one heading).
  const Vec3 preset = vec3Add(BIAS, vec3(0.0f, 0.0f, 0.005f * DEG_PER_S));
  const Quat truth = quatAboutUp(30.0f / DEG);
  HeadingObserver obs;
  obs.reset(preset, LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE);
  obs.addPeriod(restMean(truth, BIAS), 20.0f, truth);
  CHECK(obs.solve(truth));
  const Vec3 err = vec3Sub(obs.bias(), BIAS);
  METRIC("z bias error (deg/s, preset was off by 0.005)", err.z * DEG);
  METRIC("horizontal bias error (deg/s)", sqrtf(err.x * err.x + err.y * err.y) * DEG);
  CHECK_LT(fabsf(err.z) * DEG, 0.0001f);
  CHECK_NEAR(sqrtf(err.x * err.x + err.y * err.y), 0.0f, 1.0e-7f);
  CHECK_LT(fabsf(obs.pendingYaw()) * DEG, 0.01f);
}

TEST(heading_observer_period_in_progress_counts_and_short_stops_are_dropped)
{
  const Quat truth = tiltedAttitude(-70.0f);
  const Quat ahrs = ahrsWithYawError(truth, -35.0f);
  const Vec3 mean = restMean(truth, BIAS);
  HeadingObserver obs;
  obs.reset(BIAS, LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE);
  CHECK(!obs.solve(ahrs)); // nothing yet
  for (int i = 0; i < LOCALIZER_SAMPLE_RATE; i++) obs.addStationary(mean, LOCALIZER_NOMINAL_DT);
  CHECK(!obs.solve(ahrs)); // 1 s: shorter than the minimum period
  obs.endPeriod(ahrs);
  CHECK(obs.status().periods == 0);
  // the first 0.3 s of a rest are skipped (the turn may still be settling)
  for (int i = 0; i < 3 * LOCALIZER_SAMPLE_RATE; i++) obs.addStationary(mean, LOCALIZER_NOMINAL_DT);
  CHECK_NEAR(obs.currentPeriodTime(), 2.7f, 0.01f);
  CHECK(obs.solve(ahrs));
  CHECK_NEAR(obs.pendingYaw() * DEG, -35.0f, 0.01f);
  // a rotation that starts before the ZUPT detector notices must not leak into the mean
  const Vec3 turning = vec3Add(mean, vec3(0.0f, 0.0f, 2.0f * DEG_PER_S));
  for (int i = 0; i < LOCALIZER_SAMPLE_RATE / 5; i++) obs.addStationary(turning, LOCALIZER_NOMINAL_DT);
  obs.endPeriod(ahrs);
  CHECK(obs.status().periods == 1);
  CHECK_NEAR(obs.period(0).duration, 2.7f, 0.11f); // 0.2 s of turn and up to a chunk dropped
  CHECK_LT(vec3Norm(vec3Sub(obs.period(0).mean, mean)) * DEG, 1.0e-5f);
  CHECK_NEAR(obs.currentPeriodTime(), 0.0f, 1.0e-6f);
  CHECK(obs.solve(ahrs));
  CHECK_NEAR(obs.pendingYaw() * DEG, -35.0f, 0.01f);
}

TEST(heading_observer_rejects_a_rotation_as_rest)
{
  const Quat truth = tiltedAttitude(10.0f);
  HeadingObserver obs;
  obs.reset(BIAS, LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE);
  obs.addPeriod(vec3Add(restMean(truth, BIAS), vec3(0.1f * DEG_PER_S, 0.0f, 0.0f)), 5.0f, truth);
  CHECK(obs.status().periods == 0);
  CHECK(obs.status().rejectedPeriods == 1);
  obs.addPeriod(restMean(truth, BIAS), 5.0f, truth);
  CHECK(obs.status().periods == 1);
}

TEST(heading_observer_period_weight_follows_length_and_age)
{
  CHECK_NEAR(HeadingObserver::periodWeight(3.0f, 0.0f), 1.0f, 1.0e-6f);
  CHECK_NEAR(HeadingObserver::periodWeight(15.0f, 0.0f), 5.0f, 1.0e-6f);
  CHECK_NEAR(HeadingObserver::periodWeight(600.0f, 0.0f), HEADING_OBSERVER_MAX_PERIOD / 3.0f, 1.0e-6f);
  CHECK_NEAR(HeadingObserver::periodWeight(3.0f, HEADING_OBSERVER_MEMORY), expf(-1.0f), 1.0e-6f);
}

TEST(heading_observer_old_periods_lose_weight_against_a_changed_bias)
{
  // A y bias that moved by 0.003 deg/s after the first rest (same heading
  // both times, so the bias stays at the preset and the heading absorbs
  // the change). While the first rest is fresh the two are averaged; once
  // it is many memory constants old the new rest decides.
  const Quat truth = quatIdentity(); // x North, y West: the y shift is perpendicular to North
  const Vec3 later = vec3Add(BIAS, vec3(0.0f, 0.003f * DEG_PER_S, 0.0f));
  const float expectedYaw = atan2f(0.003f * DEG_PER_S, LOCALIZER_EARTH_RATE * cosf(LOCALIZER_LATITUDE_DEG / DEG)) * DEG;
  float yaw[2];
  for (int aged = 0; aged < 2; aged++)
  {
    HeadingObserver obs;
    obs.reset(BIAS, LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE);
    obs.addPeriod(restMean(truth, BIAS), 30.0f, truth);
    if (aged) obs.advance(5.0f * HEADING_OBSERVER_MEMORY);
    obs.addPeriod(restMean(truth, later), 30.0f, truth);
    CHECK(obs.solve(truth));
    yaw[aged] = fabsf(obs.pendingYaw() * DEG);
  }
  METRIC("yaw a 0.003 deg/s y shift implies (deg)", expectedYaw);
  METRIC("yaw with both rests fresh (deg)", yaw[0]);
  METRIC("yaw with the first rest aged out (deg)", yaw[1]);
  CHECK_LT(0.4f * expectedYaw, yaw[0]);
  CHECK_LT(yaw[0], 0.6f * expectedYaw);
  CHECK_LT(0.95f * expectedYaw, yaw[1]);
  CHECK_LT(yaw[1], 1.02f * expectedYaw);
}

TEST(heading_observer_keeps_the_longest_rests_when_full)
{
  // A run of brief stops must not push the 60 s rest (or the alignment)
  // out of the store: the lightest period is replaced, not the oldest.
  const Quat truth = tiltedAttitude(5.0f);
  HeadingObserver obs;
  obs.reset(BIAS, LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE);
  obs.addPeriod(restMean(truth, BIAS), 60.0f, truth);
  for (int i = 0; i < 3 * HEADING_OBSERVER_PERIODS; i++)
  {
    obs.advance(1.0f);
    obs.addPeriod(restMean(truth, BIAS), 2.5f, truth);
  }
  CHECK(obs.status().periods == HEADING_OBSERVER_PERIODS);
  bool longKept = false;
  for (int i = 0; i < obs.periodCount(); i++)
    if (obs.period(i).duration == 60.0f) longKept = true;
  CHECK(longKept);
}
