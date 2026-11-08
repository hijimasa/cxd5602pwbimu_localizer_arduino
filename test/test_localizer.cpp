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
  float finalHeadingErrorDeg = 0.0f;   // signed, against the truth
  float finalPositionErrorTrueFrame = 0.0f; // against the truth as is (needs a correct heading)
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
    r.finalHeadingErrorDeg = headErr;
    r.finalPositionErrorTrueFrame = vec3Norm(vec3Sub(st.position, truth.position));
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
  // offset tracker (FusionBias) absorbs the earth rotation (~0.0042 deg/s
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

TEST(localizer_gyro_preset_gives_true_heading)
{
  // Real-board bias (0.006, -0.005, -0.023 deg/s), larger than the
  // horizontal earth rate: the initial heading is arbitrary unless the
  // gyro bias preset from the six-orientation calibration is applied.
  SimConfig cfg = realisticNoise();
  cfg.gyroBias = vec3(1.1e-4f, -0.9e-4f, -4.1e-4f);
  Localizer without;
  const RunResult a = runSimulation(without, cfg, stationaryTrajectory(tilted()), 20.0);
  Localizer with;
  with.setGyroBiasPreset(cfg.gyroBias, true);
  const RunResult b = runSimulation(with, cfg, stationaryTrajectory(tilted()), 20.0);
  METRIC("initial heading error without preset (deg)", a.initialHeadingErrorDeg);
  METRIC("initial heading error with preset (deg)", b.initialHeadingErrorDeg);
  CHECK(with.alignment().gyroPresetUsed);
  CHECK(!without.alignment().gyroPresetUsed);
  // 15 s of white noise on the horizontal rate leaves a few degrees
  CHECK_LT(fabsf(b.initialHeadingErrorDeg), 8.0f);
  CHECK_LT(20.0f, fabsf(a.initialHeadingErrorDeg));
  CHECK_LT(b.maxTiltErrorDeg, 0.3f);
  CHECK_LT(b.finalPositionError, 0.002f);
}

TEST(localizer_drifted_gyro_preset_does_not_drift_attitude)
{
  // The preset is used for the heading only; a preset that no longer fits
  // (0.001 deg/s off, a few degrees of temperature change) must leave the
  // attitude at rest as stable as without any preset. With the heading
  // observer the heading is a live gyro-compass estimate (it follows the
  // mean of the rest, a few degrees of noise); without it the heading is
  // frozen at the alignment.
  SimConfig cfg = realisticNoise();
  cfg.gyroBias = vec3(1.1e-4f, -0.9e-4f, -4.1e-4f);
  const Vec3 drifted = vec3Add(cfg.gyroBias, vec3(1.2e-5f, -1.0e-5f, 0.8e-5f));
  Localizer reference;
  const RunResult a = runSimulation(reference, cfg, stationaryTrajectory(tilted()), 60.0);
  Localizer frozen;
  frozen.setHeadingObserver(false);
  frozen.setGyroBiasPreset(drifted, true);
  const RunResult b = runSimulation(frozen, cfg, stationaryTrajectory(tilted()), 60.0);
  Localizer live;
  live.setGyroBiasPreset(drifted, true);
  const RunResult c = runSimulation(live, cfg, stationaryTrajectory(tilted()), 180.0);
  // At a single heading the gyro-compass cannot tell the preset error from
  // the earth rate: the heading it converges to is off by the angle between
  // the horizontal parts of the earth rate and of (earth rate + error).
  const Quat truth = tilted();
  const Vec3 up = quatRotateEarthToSensor(truth, vec3(0, 0, 1));
  const Vec3 e = quatRotateEarthToSensor(truth, earthRateNwu(cfg.latitudeDeg, cfg.earthRate));
  const Vec3 rr = vec3Add(e, vec3Sub(cfg.gyroBias, drifted));
  const Vec3 eh = vec3Sub(e, vec3Scale(up, vec3Dot(e, up)));
  const Vec3 rh = vec3Sub(rr, vec3Scale(up, vec3Dot(rr, up)));
  const float limitDeg = atan2f(vec3Dot(up, vec3Cross(rh, eh)), vec3Dot(rh, eh)) * DEG;
  METRIC("heading drift without preset (deg)", a.maxHeadingDriftDeg);
  METRIC("heading drift with drifted preset, observer off (deg)", b.maxHeadingDriftDeg);
  METRIC("initial heading error with drifted preset (deg)", c.initialHeadingErrorDeg);
  METRIC("heading drift with drifted preset, observer on (deg)", c.maxHeadingDriftDeg);
  METRIC("final heading error, observer on (deg)", c.finalHeadingErrorDeg);
  METRIC("single-heading limit of this preset error (deg)", limitDeg);
  METRIC("tilt error with drifted preset (deg)", c.maxTiltErrorDeg);
  CHECK_LT(b.maxHeadingDriftDeg, a.maxHeadingDriftDeg + 0.02f);
  CHECK_LT(b.maxHeadingDriftDeg, 0.1f);
  // the 180 s estimate sits at the limit, within the noise of the mean (about 1.4 deg)
  CHECK_LT(fabsf(wrapDeg(c.finalHeadingErrorDeg - limitDeg)), 4.5f);
  CHECK_LT(c.maxHeadingDriftDeg, 8.0f);
  for (const RunResult *r : {&b, &c})
  {
    CHECK_LT(r->maxTiltErrorDeg, 0.3f);
    CHECK_LT(r->maxSpeedErrorWhileStill, 0.01f);
    CHECK_LT(r->finalPositionError, 0.002f);
  }
}

/** Rotation about the body z axis from t0, then a translation from t1 (Earth frame). */
static TrajectoryFn turnThenTranslate(Quat attitude0, double t0, double turnDuration, float angle,
                                      double t1, double moveDuration, Vec3 displacement)
{
  const TrajectoryFn turn = rotationTrajectory(attitude0, t0, turnDuration, vec3(0, 0, 1), angle);
  const Quat attitude1 = quatMultiply(attitude0, quatFromAxisAngle(vec3(0, 0, 1), angle));
  const TrajectoryFn move = translationTrajectory(attitude1, t1, moveDuration, displacement);
  return [=](double t) { return t < t1 ? turn(t) : move(t); };
}

TEST(localizer_heading_observer_resolves_east_west_after_a_turn)
{
  // The user's symptom: the y-axis preset is off by 0.006 deg/s, more than
  // the horizontal earth rate, so the alignment with x East reads the
  // mirror image (x West, 180 deg off). After a 180 deg turn and a rest the
  // observer sees both headings, separates the bias and turns the attitude
  // to the true heading (5 deg/s: 176 deg take 35 s); the path integrated
  // afterwards is right in the Earth frame, not only relative to the start.
  SimConfig cfg = realisticNoise();
  cfg.gyroBias = vec3(1.1e-4f, -0.9e-4f, -4.1e-4f);
  const Vec3 preset = vec3Add(cfg.gyroBias, vec3(0.0f, 0.006f / DEG, 0.0f));
  const Quat xEast = quatFromAxisAngle(vec3(0, 0, 1), -90.0f / DEG);
  const TrajectoryFn traj = turnThenTranslate(xEast, 25.0, 3.0, 180.0f / DEG, 80.0, 2.0, vec3(0.5f, 0, 0));
  Localizer with;
  with.setGyroBiasPreset(preset, true);
  const RunResult a = runSimulation(with, cfg, traj, 90.0);
  Localizer without;
  without.setHeadingObserver(false);
  without.setGyroBiasPreset(preset, true);
  const RunResult b = runSimulation(without, cfg, traj, 90.0);
  const HeadingObserverStatus &st = with.headingObserver().status();
  METRIC("initial heading error (deg)", a.initialHeadingErrorDeg);
  METRIC("final heading error with observer (deg)", a.finalHeadingErrorDeg);
  METRIC("final heading error without observer (deg)", b.finalHeadingErrorDeg);
  METRIC("final position error in the Earth frame, with (m)", a.finalPositionErrorTrueFrame);
  METRIC("final position error in the Earth frame, without (m)", b.finalPositionErrorTrueFrame);
  METRIC("bias error of the observer (deg/s)", vec3Norm(vec3Sub(st.bias, cfg.gyroBias)) * DEG);
  METRIC("bias error x (deg/s)", (st.bias.x - cfg.gyroBias.x) * DEG);
  METRIC("bias error y (deg/s)", (st.bias.y - cfg.gyroBias.y) * DEG);
  METRIC("bias error z (deg/s)", (st.bias.z - cfg.gyroBias.z) * DEG);
  METRIC("total yaw applied (deg)", st.yawApplied * DEG);
  METRIC("periods stored", st.periods);
  METRIC("max tilt error (deg)", a.maxTiltErrorDeg);
  CHECK_LT(150.0f, fabsf(a.initialHeadingErrorDeg));
  CHECK_LT(150.0f, fabsf(b.finalHeadingErrorDeg));
  CHECK_LT(fabsf(a.finalHeadingErrorDeg), 4.0f);
  CHECK_LT(vec3Norm(vec3Sub(st.bias, cfg.gyroBias)) * DEG, 0.0006f); // three 15-45 s means: about 0.0002 deg/s of noise
  CHECK_LT(a.finalPositionErrorTrueFrame, 0.08f); // a few cm from the 180 deg turn itself (see localizer_rotation_then_rest)
  CHECK_LT(0.9f, b.finalPositionErrorTrueFrame); // the mirror image path: about 1 m off
  CHECK_LT(a.maxTiltErrorDeg, 0.5f);
  CHECK(a.stationaryAtEnd);
  CHECK(st.periods >= 2);
}

TEST(localizer_heading_correction_leaves_the_position_alone)
{
  // Design decision: the correction turns the attitude only. A robot acting
  // on the position must not see it move while the sensor rests, so the
  // path integrated before the correction is not rotated into the new
  // frame (it stays where it was; the new frame applies from then on).
  SimConfig cfg = realisticNoise();
  cfg.gyroBias = vec3(1.1e-4f, -0.9e-4f, -4.1e-4f);
  const Vec3 preset = vec3Add(cfg.gyroBias, vec3(0.0f, 0.006f / DEG, 0.0f));
  const Quat xEast = quatFromAxisAngle(vec3(0, 0, 1), -90.0f / DEG);
  // 0.5 m move while the heading is still the mirror image, then the turn
  const TrajectoryFn move = translationTrajectory(xEast, 17.0, 2.0, vec3(0.5f, 0, 0));
  const TrajectoryFn turn = rotationTrajectory(xEast, 25.0, 3.0, vec3(0, 0, 1), 180.0f / DEG);
  const TrajectoryFn traj = [=](double t) {
    TruePose p = t < 25.0 ? move(t) : turn(t);
    if (t >= 25.0) p.position = vec3(0.5f, 0, 0);
    return p;
  };
  Localizer loc;
  loc.setGyroBiasPreset(preset, true);
  ImuSimulator sim(cfg);
  Vec3 posBeforeTurn = vec3Zero(), posAtRest = vec3Zero();
  float maxMoveAtRest = 0.0f, headingBefore = 0.0f, headingAfter = 0.0f;
  bool haveRest = false;
  const uint64_t N = (uint64_t)(75.0 * LOCALIZER_SAMPLE_RATE);
  for (uint64_t k = 0; k < N; k++)
  {
    TruePose truth;
    const ImuSample s = sim.sample(k, traj, &truth);
    if (!loc.update(s)) continue;
    const double t = (double)k / LOCALIZER_SAMPLE_RATE;
    const LocalizerState &st = loc.state();
    if (t < 25.0)
    {
      posBeforeTurn = st.position;
      headingBefore = headingErrorDeg(st.quaternion, truth.attitude);
    }
    else if (t > 31.0 && st.stationary) // 3 s after the turn ended: the ZUPT blend is over
    {
      if (!haveRest)
      {
        posAtRest = st.position;
        haveRest = true;
      }
      const float d = vec3Norm(vec3Sub(st.position, posAtRest));
      if (d > maxMoveAtRest) maxMoveAtRest = d;
      headingAfter = headingErrorDeg(st.quaternion, truth.attitude);
    }
  }
  METRIC("heading error before the turn (deg)", headingBefore);
  METRIC("heading error at the end of the rest (deg)", headingAfter);
  METRIC("position movement during the rest (m)", maxMoveAtRest);
  METRIC("position change through the turn (m)", vec3Norm(vec3Sub(posAtRest, posBeforeTurn)));
  CHECK(haveRest);
  CHECK_LT(150.0f, fabsf(headingBefore));
  CHECK_LT(fabsf(headingAfter), 5.0f);
  CHECK_LT(maxMoveAtRest, 1.0e-4f);                                   // nothing moves at rest
  CHECK_LT(vec3Norm(vec3Sub(posAtRest, posBeforeTurn)), 0.06f);       // the path is not rotated (that would move it by about 1 m); a few cm come from the turn itself
  CHECK_LT(0.45f, vec3Norm(posAtRest));                               // the 0.5 m move is kept as integrated
}

TEST(localizer_heading_observer_long_rest_refines_the_heading)
{
  // Exact preset, 120 s at rest: the heading must end closer to the truth
  // than the 15 s alignment noise allows and the attitude/position at rest
  // must stay as good as without the observer.
  SimConfig cfg = realisticNoise();
  cfg.gyroBias = vec3(1.1e-4f, -0.9e-4f, -4.1e-4f);
  Localizer loc;
  loc.setGyroBiasPreset(cfg.gyroBias, true);
  const RunResult r = runSimulation(loc, cfg, stationaryTrajectory(tilted()), 120.0);
  const HeadingObserverStatus &st = loc.headingObserver().status();
  METRIC("initial heading error (deg)", r.initialHeadingErrorDeg);
  METRIC("final heading error (deg)", r.finalHeadingErrorDeg);
  METRIC("max heading change (deg)", r.maxHeadingDriftDeg);
  METRIC("bias error of the observer (deg/s)", vec3Norm(vec3Sub(st.bias, cfg.gyroBias)) * DEG);
  METRIC("earth rate seen (deg/h, expected 15.04)", st.rateMagnitude * DEG * 3600.0f);
  METRIC("max tilt error (deg)", r.maxTiltErrorDeg);
  CHECK_LT(fabsf(r.finalHeadingErrorDeg), 2.0f);
  CHECK_LT(r.maxHeadingDriftDeg, 6.0f);
  CHECK_LT(r.maxTiltErrorDeg, 0.3f);
  CHECK_LT(r.maxSpeedErrorWhileStill, 0.01f);
  CHECK_LT(r.finalPositionError, 0.002f);
  CHECK_LT(vec3Norm(vec3Sub(st.bias, cfg.gyroBias)) * DEG, 0.0005f);
  CHECK(r.stationaryAtEnd);
}

TEST(localizer_heading_observer_is_idle_without_a_preset)
{
  // No preset: nothing to anchor the bias, the observer must not act and
  // the run must be identical to the observer-less one.
  Localizer a, b;
  b.setHeadingObserver(false);
  const RunResult ra = runSimulation(a, realisticNoise(), translationTrajectory(LEVEL, 17.0, 2.0, vec3(1, 0, 0)), 25.0);
  const RunResult rb = runSimulation(b, realisticNoise(), translationTrajectory(LEVEL, 17.0, 2.0, vec3(1, 0, 0)), 25.0);
  CHECK(!a.headingObserverActive());
  CHECK(a.headingObserver().status().periods == 0);
  CHECK_NEAR(vec3Norm(vec3Sub(ra.last.position, rb.last.position)), 0.0f, 0.0f);
  CHECK_NEAR(ra.last.quaternion.w, rb.last.quaternion.w, 0.0f);
}
