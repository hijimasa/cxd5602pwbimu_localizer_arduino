#include "test_framework.h"
#include "imu_simulator.h"
#include <vector>

static const float DEG = 180.0f / LOCALIZER_PI;

struct RunResult
{
  LocalizerState last;
  TruePose lastTruth;
  float maxTiltErrorDeg = 0.0f;    // after calibration
  float maxHeadingDriftDeg = 0.0f; // yaw error change since the first running sample
  float initialHeadingErrorDeg = 0.0f; // arbitrary unless north-finding is accurate
  float finalTiltErrorDeg = 0.0f;
  float finalHeadingDriftDeg = 0.0f;
  float finalPositionError = 0.0f; // in the initial-heading-aligned frame (see below)
  float maxSpeedErrorWhileStill = 0.0f;
  bool stationaryAtEnd = false;
  uint32_t runningSamples = 0;
  uint32_t stationarySamples = 0;
};

static float tiltErrorDeg(Quat est, Quat truth)
{
  return vec3AngleBetween(quatRotateEarthToSensor(est, vec3(0, 0, 1)),
                          quatRotateEarthToSensor(truth, vec3(0, 0, 1))) * DEG;
}

static float headingErrorDeg(Quat est, Quat truth)
{
  // signed angle between the horizontal projections of the sensor x axis
  Vec3 xe = quatRotateSensorToEarth(est, vec3(1, 0, 0));
  Vec3 xt = quatRotateSensorToEarth(truth, vec3(1, 0, 0));
  xe.z = 0.0f; xt.z = 0.0f;
  return atan2f(vec3Cross(xt, xe).z, vec3Dot(xt, xe)) * DEG;
}

/** Rotate a vector about the Earth z axis by angle (deg). */
static Vec3 rotateZ(Vec3 v, float deg)
{
  const float c = cosf(deg / DEG), s = sinf(deg / DEG);
  return vec3(c * v.x - s * v.y, s * v.x + c * v.y, v.z);
}

static float wrapDeg(float a)
{
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

/** Run the localizer on a trajectory for `seconds` (including calibration). */
static RunResult runSimulation(Localizer &loc, const SimConfig &cfg, const TrajectoryFn &traj, double seconds)
{
  ImuSimulator sim(cfg);
  RunResult r;
  const uint64_t N = (uint64_t)(seconds * LOCALIZER_SAMPLE_RATE);
  for (uint64_t k = 0; k < N; k++)
  {
    TruePose truth;
    const ImuSample s = sim.sample(k, traj, &truth);
    if (!loc.update(s)) continue;
    const LocalizerState &st = loc.state();
    const float tilt = tiltErrorDeg(st.quaternion, truth.attitude);
    const float headErr = headingErrorDeg(st.quaternion, truth.attitude);
    if (r.runningSamples == 0) r.initialHeadingErrorDeg = headErr;
    r.runningSamples++;
    const float head = fabsf(wrapDeg(headErr - r.initialHeadingErrorDeg));
    if (tilt > r.maxTiltErrorDeg) r.maxTiltErrorDeg = tilt;
    if (head > r.maxHeadingDriftDeg) r.maxHeadingDriftDeg = head;
    if (st.stationary) r.stationarySamples++;
    if (vec3Norm(truth.velocity) == 0.0f)
    {
      const float e = vec3Norm(vec3Sub(st.velocity, truth.velocity));
      if (e > r.maxSpeedErrorWhileStill) r.maxSpeedErrorWhileStill = e;
    }
    r.last = st;
    r.lastTruth = truth;
    r.finalTiltErrorDeg = tilt;
    r.finalHeadingDriftDeg = head;
    // Without a magnetometer the absolute heading is only as good as the
    // gyro-compass alignment (gyro bias vs. earth rate). Position accuracy
    // is therefore judged after removing the initial heading error.
    r.finalPositionError = vec3Norm(vec3Sub(rotateZ(st.position, -r.initialHeadingErrorDeg), truth.position));
    r.stationaryAtEnd = st.stationary;
  }
  return r;
}

static SimConfig realisticNoise()
{
  SimConfig cfg;
  // per-sample white noise from the datasheet densities
  cfg.accelNoiseStd = ACCEL_NOISE_DENSITY * sqrtf((float)LOCALIZER_SAMPLE_RATE);
  cfg.gyroNoiseStd = GYRO_NOISE_DENSITY_RAD * sqrtf((float)LOCALIZER_SAMPLE_RATE);
  cfg.accelBias = vec3(0.02f, -0.015f, 0.03f);
  cfg.gyroBias = vec3(2.0e-4f, -1.5e-4f, 1.0e-4f); // ~0.01 deg/s
  return cfg;
}

static const Quat LEVEL = {1.0f, 0.0f, 0.0f, 0.0f};
static Quat tilted()
{
  const Quat roll = quatFromAxisAngle(vec3(1, 0, 0), 30.0f / DEG);
  const Quat pitch = quatFromAxisAngle(vec3(0, 1, 0), -20.0f / DEG);
  const Quat yaw = quatFromAxisAngle(vec3(0, 0, 1), 70.0f / DEG);
  return quatMultiply(yaw, quatMultiply(pitch, roll));
}

TEST(localizer_calibration_phase_length)
{
  Localizer loc;
  SimConfig cfg;
  ImuSimulator sim(cfg);
  const TrajectoryFn traj = stationaryTrajectory(LEVEL);
  uint64_t k = 0;
  for (; k < (uint64_t)LOCALIZER_CALIBRATION_SAMPLES; k++)
  {
    CHECK(loc.phase() == Localizer::CALIBRATING);
    CHECK(!loc.update(sim.sample(k, traj)));
  }
  CHECK(loc.phase() == Localizer::RUNNING);
  CHECK(loc.update(sim.sample(k, traj)));
  CHECK(loc.diagnostics().samplesProcessed == 1);
}

TEST(localizer_stationary_level_with_noise_and_bias)
{
  Localizer loc;
  const RunResult r = runSimulation(loc, realisticNoise(), stationaryTrajectory(LEVEL), 30.0);
  METRIC("max tilt error (deg)", r.maxTiltErrorDeg);
  METRIC("max speed error while still (m/s)", r.maxSpeedErrorWhileStill);
  METRIC("final position error (m)", r.finalPositionError);
  CHECK(r.stationaryAtEnd);
  CHECK_LT(r.maxTiltErrorDeg, 0.3f);
  CHECK_LT(r.maxSpeedErrorWhileStill, 0.01f);
  CHECK_LT(r.finalPositionError, 0.002f);
  CHECK(loc.diagnostics().dtAnomalies == 0);
}

TEST(localizer_tilted_start_has_no_initial_attitude_error)
{
  // Regression for the inverted initial quaternion: with a 30/20 deg tilt
  // the old code started with a large attitude error. Ideal sensors.
  Localizer loc;
  SimConfig cfg;
  const RunResult r = runSimulation(loc, cfg, stationaryTrajectory(tilted()), 20.0);
  METRIC("max tilt error after calibration (deg)", r.maxTiltErrorDeg);
  METRIC("initial heading error (deg)", r.initialHeadingErrorDeg);
  METRIC("max heading drift after calibration (deg)", r.maxHeadingDriftDeg);
  CHECK_LT(r.maxTiltErrorDeg, 0.05f);
  // ideal gyro: north-finding from the earth rate is exact
  CHECK_LT(fabsf(r.initialHeadingErrorDeg), 0.1f);
  CHECK_LT(r.maxHeadingDriftDeg, 0.1f);
  CHECK(r.stationaryAtEnd);
  CHECK_LT(r.finalPositionError, 1e-3f);
}

TEST(localizer_tilted_start_with_noise)
{
  Localizer loc;
  const RunResult r = runSimulation(loc, realisticNoise(), stationaryTrajectory(tilted()), 30.0);
  METRIC("max tilt error (deg)", r.maxTiltErrorDeg);
  METRIC("max speed error while still (m/s)", r.maxSpeedErrorWhileStill);
  CHECK_LT(r.maxTiltErrorDeg, 0.3f);
  CHECK_LT(r.maxSpeedErrorWhileStill, 0.01f);
  CHECK_LT(r.finalPositionError, 0.002f);
}

TEST(localizer_earth_rate_compensation_keeps_gyro_offset_clean)
{
  // Stationary, noisy but bias-free gyro. Without compensation the run-time
  // offset tracker (FusionOffset) absorbs the earth rotation (~0.0042 deg/s
  // in total) as if it were sensor bias, which becomes wrong after any
  // rotation. With compensation the tracked offset must stay near zero.
  SimConfig cfg = realisticNoise();
  cfg.gyroBias = vec3Zero();
  cfg.accelBias = vec3Zero();
  Localizer with;
  runSimulation(with, cfg, stationaryTrajectory(tilted()), 75.0);
  Localizer without;
  without.setEarthRateCompensation(false);
  runSimulation(without, cfg, stationaryTrajectory(tilted()), 75.0);
  const float offWith = vec3Norm(with.gyroOffsetEstimate()) * DEG;
  const float offWithout = vec3Norm(without.gyroOffsetEstimate()) * DEG;
  const float earthRateDeg = LOCALIZER_EARTH_RATE * DEG;
  METRIC("tracked gyro offset with compensation (deg/s)", offWith);
  METRIC("tracked gyro offset without compensation (deg/s)", offWithout);
  METRIC("earth rate (deg/s)", earthRateDeg);
  CHECK_LT(offWith, 0.25f * earthRateDeg);
  CHECK_LT(0.75f * earthRateDeg, offWithout);
  CHECK_LT(offWithout, 1.25f * earthRateDeg);
}

TEST(localizer_timestamp_wraparound_is_transparent)
{
  // Same trajectory with and without a 32-bit timestamp wrap during the run:
  // results must be bit-identical and no dt anomaly must be reported.
  SimConfig a = realisticNoise();
  SimConfig b = a;
  b.startTimestamp = 0xFFFFFFFFu - (uint32_t)(20.0f * LOCALIZER_TIMESTAMP_HZ); // wraps at t = 20 s
  Localizer la, lb;
  const RunResult ra = runSimulation(la, a, stationaryTrajectory(tilted()), 25.0);
  const RunResult rb = runSimulation(lb, b, stationaryTrajectory(tilted()), 25.0);
  CHECK(la.diagnostics().dtAnomalies == 0);
  CHECK(lb.diagnostics().dtAnomalies == 0);
  CHECK(ra.last.position.x == rb.last.position.x);
  CHECK(ra.last.quaternion.w == rb.last.quaternion.w);
  CHECK(ra.last.velocity.z == rb.last.velocity.z);
}

TEST(localizer_lost_samples_are_reported_and_dt_stretches)
{
  Localizer loc;
  SimConfig cfg;
  ImuSimulator sim(cfg);
  const TrajectoryFn traj = stationaryTrajectory(LEVEL);
  uint64_t k = 0;
  for (; k < (uint64_t)LOCALIZER_CALIBRATION_SAMPLES + 100; k++) loc.update(sim.sample(k, traj));
  CHECK(loc.diagnostics().dtAnomalies == 0);
  k += 2; // skip two samples
  loc.update(sim.sample(k, traj));
  CHECK(loc.diagnostics().dtAnomalies == 1);
  CHECK_NEAR(loc.state().dt, 3.0f * LOCALIZER_NOMINAL_DT, 1e-6);
  k += 1000; // huge gap is clamped
  loc.update(sim.sample(k, traj));
  CHECK(loc.diagnostics().dtAnomalies == 2);
  CHECK_NEAR(loc.state().dt, LOCALIZER_MAX_DT, 1e-6);
  CHECK_NEAR(loc.diagnostics().maxDt, LOCALIZER_MAX_DT, 1e-6);
}

TEST(localizer_rotation_then_rest)
{
  // 90 deg yaw rotation over 2 s starting at t = 17 s, then rest.
  Localizer loc;
  const Quat q0 = tilted();
  const RunResult r = runSimulation(loc, realisticNoise(), rotationTrajectory(q0, 17.0, 2.0, vec3(0, 0, 1), 90.0f / DEG), 25.0);
  METRIC("final tilt error (deg)", r.finalTiltErrorDeg);
  METRIC("final heading drift (deg)", r.finalHeadingDriftDeg);
  METRIC("final position error (m)", r.finalPositionError);
  CHECK_LT(r.finalTiltErrorDeg, 0.5f);
  CHECK_LT(r.finalHeadingDriftDeg, 1.0f);
  CHECK(r.stationaryAtEnd);
  CHECK_LT(r.finalPositionError, 0.03f);
}

static RunResult translationRun(Localizer &loc, double duration, float distance, double rest = 3.0)
{
  return runSimulation(loc, realisticNoise(), translationTrajectory(LEVEL, 17.0, duration, vec3(distance, 0, 0)), 17.0 + duration + rest);
}

TEST(localizer_smooth_translation_2s)
{
  // 1 m along Earth X in 2 s with a minimum-jerk profile (peak ~1.4 m/s^2).
  // Around the mid point the acceleration is ~0 for ~0.1 s: the ZUPT must
  // not fire there (speed-dependent confirmation time).
  Localizer loc;
  const RunResult r = translationRun(loc, 2.0, 1.0f);
  METRIC("final position error (m)", r.finalPositionError);
  METRIC("estimated distance (m)", vec3Norm(r.last.position));
  METRIC("max tilt error during run (deg)", r.maxTiltErrorDeg);
  CHECK(r.stationaryAtEnd);
  CHECK_LT(r.finalPositionError, 0.01f);
}

TEST(localizer_slow_translation_4s)
{
  // 1 m in 4 s (peak ~0.36 m/s^2): close to the mean-acceleration threshold,
  // with a ~0.4 s near-zero acceleration phase in the middle.
  Localizer loc;
  const RunResult r = translationRun(loc, 4.0, 1.0f);
  METRIC("final position error (m)", r.finalPositionError);
  METRIC("estimated distance (m)", vec3Norm(r.last.position));
  CHECK(r.stationaryAtEnd);
  CHECK_LT(r.finalPositionError, 0.05f);
}

TEST(localizer_abrupt_translation)
{
  // 0.3 m along Earth X in 0.5 s (peak ~6.9 m/s^2), then rest.
  Localizer loc;
  const RunResult r = translationRun(loc, 0.5, 0.3f);
  METRIC("final position error (m)", r.finalPositionError);
  METRIC("estimated distance (m)", vec3Norm(r.last.position));
  METRIC("max tilt error during run (deg)", r.maxTiltErrorDeg);
  CHECK(r.stationaryAtEnd);
  CHECK_NEAR(r.last.velocity.x, 0.0f, 1e-6);
  CHECK_LT(r.finalPositionError, 0.02f);
}

TEST(localizer_long_translation_3s_2m)
{
  Localizer loc;
  const RunResult r = translationRun(loc, 3.0, 2.0f);
  METRIC("final position error (m)", r.finalPositionError);
  METRIC("estimated distance (m)", vec3Norm(r.last.position));
  CHECK(r.stationaryAtEnd);
  CHECK_LT(r.finalPositionError, 0.03f);
}

TEST(localizer_gain_scheduling_reduces_motion_tilt_error)
{
  // With a fixed gain the accelerometer feedback tilts the attitude towards
  // the apparent gravity during translation; scheduling must reduce that.
  Localizer with;
  const RunResult a = translationRun(with, 2.0, 1.0f);
  Localizer without;
  without.setGainScheduling(false);
  const RunResult b = translationRun(without, 2.0, 1.0f);
  METRIC("max tilt error with scheduling (deg)", a.maxTiltErrorDeg);
  METRIC("max tilt error without scheduling (deg)", b.maxTiltErrorDeg);
  METRIC("position error with scheduling (m)", a.finalPositionError);
  METRIC("position error without scheduling (m)", b.finalPositionError);
  CHECK_LT(a.maxTiltErrorDeg * 2.0f, b.maxTiltErrorDeg);
  CHECK_LT(a.finalPositionError, b.finalPositionError);
}

TEST(localizer_retroactive_zupt_correction_reduces_position_error)
{
  Localizer with;
  const RunResult a = translationRun(with, 2.0, 1.0f);
  Localizer without;
  without.setRetroactiveCorrection(false);
  const RunResult b = translationRun(without, 2.0, 1.0f);
  METRIC("position error with correction (m)", a.finalPositionError);
  METRIC("position error without correction (m)", b.finalPositionError);
  METRIC("velocity error at ZUPT (m/s)", vec3Norm(with.integrator().lastVelocityError()));
  METRIC("motion segment duration (s)", with.integrator().lastSegmentDuration());
  CHECK(with.integrator().correctionCount() >= 1);
  CHECK_LT(a.finalPositionError * 3.0f, b.finalPositionError);
}

TEST(localizer_position_output_has_no_step_at_zupt)
{
  // The blended correction must keep the output position continuous: the
  // largest per-sample position change after the stop must be far below
  // the total correction.
  Localizer loc;
  SimConfig cfg = realisticNoise();
  ImuSimulator sim(cfg);
  const double t0 = 17.0, dur = 2.0;
  const TrajectoryFn traj = translationTrajectory(LEVEL, t0, dur, vec3(1.0f, 0, 0));
  Vec3 prev = vec3Zero();
  float maxStepAfterStop = 0.0f;
  bool first = true;
  for (uint64_t k = 0; k < (uint64_t)(22.0 * LOCALIZER_SAMPLE_RATE); k++)
  {
    const double t = (double)k / LOCALIZER_SAMPLE_RATE;
    if (!loc.update(sim.sample(k, traj))) continue;
    const Vec3 p = loc.state().position;
    if (!first && t > t0 + dur)
    {
      const float step = vec3Norm(vec3Sub(p, prev));
      if (step > maxStepAfterStop) maxStepAfterStop = step;
    }
    prev = p;
    first = false;
  }
  const float total = vec3Norm(loc.integrator().lastCorrection());
  METRIC("total correction (m)", total);
  METRIC("max per-sample position step after stop (m)", maxStepAfterStop);
  CHECK(total > 0.0f);
  CHECK_LT(maxStepAfterStop, total * 2.0f * LOCALIZER_NOMINAL_DT / ZUPT_CORRECTION_BLEND_TIME + 1e-4f);
  CHECK_NEAR(vec3Norm(loc.integrator().pendingCorrection()), 0.0f, 1e-6);
}

TEST(localizer_stop_is_confirmed_quickly)
{
  // After a genuine stop the estimated speed is small, so the fast
  // confirmation applies: stationary within ~0.1 s after the motion ends.
  Localizer loc;
  SimConfig cfg = realisticNoise();
  ImuSimulator sim(cfg);
  const double t0 = 17.0, dur = 1.0;
  const TrajectoryFn traj = translationTrajectory(LEVEL, t0, dur, vec3(0.5f, 0, 0));
  double firstStationaryAfterStop = -1.0;
  for (uint64_t k = 0; k < (uint64_t)(20.0 * LOCALIZER_SAMPLE_RATE); k++)
  {
    const double t = (double)k / LOCALIZER_SAMPLE_RATE;
    if (!loc.update(sim.sample(k, traj))) continue;
    if (t > t0 + dur && loc.state().stationary && firstStationaryAfterStop < 0)
    {
      firstStationaryAfterStop = t - (t0 + dur);
    }
  }
  METRIC("stationary confirmed after stop (s)", firstStationaryAfterStop);
  CHECK(firstStationaryAfterStop >= 0.0);
  CHECK_LT(firstStationaryAfterStop, 0.15);
}

TEST(localizer_reset_returns_to_calibration)
{
  Localizer loc;
  SimConfig cfg;
  runSimulation(loc, cfg, stationaryTrajectory(LEVEL), 16.0);
  CHECK(loc.phase() == Localizer::RUNNING);
  loc.reset();
  CHECK(loc.phase() == Localizer::CALIBRATING);
  CHECK(loc.calibrationProgress() == 0);
  CHECK(loc.diagnostics().samplesProcessed == 0);
}
