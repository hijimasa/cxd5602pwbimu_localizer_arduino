/**
 * Regression tests on real recordings from the device (test/data/ *.bin,
 * recorded with tools/record_raw.py). Each recording starts with the 15 s
 * stationary calibration. The bounds document the achieved accuracy;
 * tighten them as the algorithm improves.
 */
#include "test_framework.h"
#include "raw_log.h"
#include <deque>

struct RecordingResult
{
  bool loaded = false;
  uint32_t imuFrames = 0, positionFrames = 0, dtAnomalies = 0, running = 0, stationary = 0;
  float maxSpeed = 0.0f;
  float maxDevicePosDiff = 0.0f;
  uint32_t deviceCompared = 0;
  Vec3 finalPosition = {0, 0, 0};
  float maxDisplacement = 0.0f;   // largest |p| reached (m)
  Vec3 destination = {0, 0, 0};   // position when the second long motion segment starts
  float finalTiltDeg = 0.0f;      // tilt of the sensor at the end
  int motionSegments = 0;         // motion segments longer than 0.2 s
};

static float tiltDeg(Quat q)
{
  const Vec3 up = quatRotateEarthToSensor(q, vec3(0, 0, 1));
  return vec3AngleBetween(up, quatRotateEarthToSensor(quatIdentity(), vec3(0, 0, 1))) * 180.0f / LOCALIZER_PI;
}

static RecordingResult runRecording(const char *path)
{
  RecordingResult r;
  std::vector<uint8_t> data;
  if (!readFile(path, data)) return r;
  r.loaded = true;
  size_t bad = 0;
  const std::vector<Frame> frames = parse(data, bad);
  Localizer loc;
  std::deque<LocalizerState> history;
  bool prevStationary = true;
  double segmentStart = 0.0;
  for (const Frame &fr : frames)
  {
    if (fr.type == RAW_LOG_TYPE_IMU)
    {
      ImuRawData_t raw;
      std::memcpy(&raw, fr.payload.data(), sizeof(raw));
      r.imuFrames++;
      ImuSample s = {raw.timestamp, raw.temp, vec3(raw.ax, raw.ay, raw.az), vec3(raw.gx, raw.gy, raw.gz)};
      if (!loc.update(s)) continue;
      const LocalizerState &st = loc.state();
      r.running++;
      history.push_back(st);
      if (history.size() > 4096) history.pop_front();
      if (st.stationary) r.stationary++;
      const float sp = vec3Norm(st.velocity);
      if (sp > r.maxSpeed) r.maxSpeed = sp;
      const float d = vec3Norm(st.position);
      if (d > r.maxDisplacement) r.maxDisplacement = d;
      const double t = (double)r.running / LOCALIZER_SAMPLE_RATE;
      if (prevStationary && !st.stationary)
      {
        segmentStart = t;
        if (r.motionSegments == 1) r.destination = st.position;
      }
      if (!prevStationary && st.stationary && t - segmentStart > 0.2) r.motionSegments++;
      prevStationary = st.stationary;
      r.finalPosition = st.position;
      r.finalTiltDeg = tiltDeg(st.quaternion);
    }
    else if (fr.type == RAW_LOG_TYPE_POSITION)
    {
      PositionData_t dev;
      std::memcpy(&dev, fr.payload.data(), sizeof(dev));
      r.positionFrames++;
      for (auto it = history.rbegin(); it != history.rend(); ++it)
      {
        if (it->timestamp != dev.timestamp) continue;
        const float d = vec3Norm(vec3Sub(vec3(dev.position[0], dev.position[1], dev.position[2]), it->position));
        if (d > r.maxDevicePosDiff) r.maxDevicePosDiff = d;
        r.deviceCompared++;
        break;
      }
    }
  }
  r.dtAnomalies = loc.diagnostics().dtAnomalies;
  return r;
}

static void commonChecks(const RecordingResult &r)
{
  CHECK(r.imuFrames > 10000);
  CHECK(r.dtAnomalies == 0);
  CHECK(r.deviceCompared > 0);
  // Only identical to the host when the recording was made with the same
  // algorithm version; kept as a gross-error check (parsing, frame order).
  METRIC("device vs host max position diff (m)", r.maxDevicePosDiff);
  CHECK_LT(r.maxDevicePosDiff, 0.05f);
}

TEST(recording_desk_move_30cm_round_trip)
{
  // Pushed 30 cm along the desk, rested, pushed back to the start.
  const RecordingResult r = runRecording("data/desk_move_30cm.bin");
  if (!r.loaded)
  {
    std::printf("    (data/desk_move_30cm.bin not found, skipped)\n");
    return;
  }
  commonChecks(r);
  METRIC("max displacement (m)", r.maxDisplacement);
  METRIC("closure error (m)", vec3Norm(r.finalPosition));
  METRIC("max speed (m/s)", r.maxSpeed);
  METRIC("motion segments", r.motionSegments);
  CHECK(r.motionSegments >= 2);
  CHECK_LT(0.28f, r.maxDisplacement);
  CHECK_LT(r.maxDisplacement, 0.33f);
  CHECK_LT(vec3Norm(r.finalPosition), 0.01f);
}

TEST(recording_pick_and_place_1m_round_trip)
{
  // Lifted, carried with ~90 deg of rotation (rates up to 230 deg/s) to a
  // place 70 cm away horizontally and about 45 cm lower, rested, then
  // carried back to the exact starting position.
  const RecordingResult r = runRecording("data/pick_and_place_1m.bin");
  if (!r.loaded)
  {
    std::printf("    (data/pick_and_place_1m.bin not found, skipped)\n");
    return;
  }
  commonChecks(r);
  const float horizontal = sqrtf(r.destination.x * r.destination.x + r.destination.y * r.destination.y);
  METRIC("destination horizontal distance (m, truth 0.70)", horizontal);
  METRIC("destination height (m, truth about -0.45)", r.destination.z);
  METRIC("closure error (m)", vec3Norm(r.finalPosition));
  METRIC("max speed (m/s)", r.maxSpeed);
  METRIC("final tilt (deg)", r.finalTiltDeg);
  CHECK(r.motionSegments >= 2);
  CHECK_LT(0.60f, horizontal);
  CHECK_LT(horizontal, 0.80f);
  CHECK_LT(-0.55f, r.destination.z);
  CHECK_LT(r.destination.z, -0.35f);
  CHECK_LT(vec3Norm(r.finalPosition), 0.02f);
  CHECK_LT(r.finalTiltDeg, 1.0f);
}
