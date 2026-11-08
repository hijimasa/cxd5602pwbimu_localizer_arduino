/**
 * @file replay.cpp
 * @brief Replay a raw log recorded with tools/record_raw.py through the
 *        localizer on the host.
 *
 * Usage: replay LOG.bin [--csv OUT.csv] [--calib-seconds S] [--no-earth-rate]
 *                       [--no-gain-schedule] [--no-retro] [--no-bias-learning]
 *                       [--no-preset] [--preset BX BY BZ] [--gyro-preset GX GY GZ] [--no-heading-observer]
 *
 * --preset takes the accelerometer bias in m/s^2, --gyro-preset the gyro
 * bias in deg/s; both override the values stored in the recording.
 *
 * Prints run statistics and, if the log also contains the position frames
 * produced on the device, the maximum deviation between the device estimate
 * and the host replay (both run the same code, so it should be tiny).
 */
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <string>
#include <deque>
#include "raw_log.h"

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::fprintf(stderr, "usage: %s LOG.bin [--csv OUT.csv] [--calib-seconds S] [--no-earth-rate] [--no-gain-schedule] [--no-retro] [--no-bias-learning] [--no-preset] [--preset BX BY BZ] [--gyro-preset GX GY GZ] [--no-heading-observer]\n", argv[0]);
    return 2;
  }
  const char *csvPath = nullptr;
  bool noPreset = false, accelPresetOverride = false, gyroPresetOverride = false;
  Localizer loc;
  for (int i = 2; i < argc; i++)
  {
    if (!std::strcmp(argv[i], "--csv") && i + 1 < argc) csvPath = argv[++i];
    else if (!std::strcmp(argv[i], "--calib-seconds") && i + 1 < argc) loc.setCalibrationSeconds((float)std::atof(argv[++i]));
    else if (!std::strcmp(argv[i], "--no-earth-rate")) loc.setEarthRateCompensation(false);
    else if (!std::strcmp(argv[i], "--no-gain-schedule")) loc.setGainScheduling(false);
    else if (!std::strcmp(argv[i], "--no-retro")) loc.setRetroactiveCorrection(false);
    else if (!std::strcmp(argv[i], "--no-bias-learning")) loc.setBiasLearning(false);
    else if (!std::strcmp(argv[i], "--no-preset")) noPreset = true;
    else if (!std::strcmp(argv[i], "--no-heading-observer")) loc.setHeadingObserver(false);
    else if (!std::strcmp(argv[i], "--preset") && i + 3 < argc)
    {
      const float x = (float)std::atof(argv[++i]), y = (float)std::atof(argv[++i]), z = (float)std::atof(argv[++i]);
      loc.setAccelBiasPreset(vec3(x, y, z));
      accelPresetOverride = true;
    }
    else if (!std::strcmp(argv[i], "--gyro-preset") && i + 3 < argc)
    {
      const float x = (float)std::atof(argv[++i]), y = (float)std::atof(argv[++i]), z = (float)std::atof(argv[++i]);
      loc.setGyroBiasPreset(vec3Scale(vec3(x, y, z), LOCALIZER_PI / 180.0f), true);
      gyroPresetOverride = true;
    }
  }

  std::vector<uint8_t> data;
  if (!readFile(argv[1], data))
  {
    std::perror(argv[1]);
    return 1;
  }

  size_t bad = 0;
  const std::vector<Frame> frames = parse(data, bad);

  FILE *csv = csvPath ? std::fopen(csvPath, "w") : nullptr;
  if (csv) std::fprintf(csv, "t,stationary,qw,qx,qy,qz,vx,vy,vz,px,py,pz,ax,ay,az,gx,gy,gz,gain\n");

  uint32_t imuCount = 0, posCount = 0, running = 0, stationaryCount = 0;
  uint32_t firstTs = 0;
  bool haveFirst = false;
  float maxSpeed = 0.0f, maxDevicePosDiff = 0.0f, maxDeviceQuatDiff = 0.0f;
  uint32_t deviceCompared = 0;
  Vec3 lastPos = vec3Zero();
  LocalizerState lastState = {};
  int outputDivider = 0;
  // The device writes the position frame for sample T when MainCore
  // receives it, i.e. a few samples after the IMU frame with timestamp T.
  // Keep a short history of host states to match by timestamp.
  std::deque<LocalizerState> history;

  for (const Frame &fr : frames)
  {
    if (fr.type == RAW_LOG_TYPE_IMU)
    {
      ImuRawData_t raw;
      std::memcpy(&raw, fr.payload.data(), sizeof(raw));
      if (!haveFirst) { firstTs = raw.timestamp; haveFirst = true; }
      imuCount++;
      ImuSample s;
      s.timestamp = raw.timestamp;
      s.temp = raw.temp;
      s.accel = vec3(raw.ax, raw.ay, raw.az);
      s.gyro = vec3(raw.gx, raw.gy, raw.gz);
      if (!loc.update(s)) continue;
      running++;
      const LocalizerState &st = loc.state();
      lastState = st;
      history.push_back(st);
      if (history.size() > 4096) history.pop_front();
      if (st.stationary) stationaryCount++;
      const float sp = vec3Norm(st.velocity);
      if (sp > maxSpeed) maxSpeed = sp;
      lastPos = st.position;
      if (csv && (++outputDivider % (LOCALIZER_SAMPLE_RATE / 30) == 0))
      {
        const double t = (double)(uint32_t)(st.timestamp - firstTs) / LOCALIZER_TIMESTAMP_HZ;
        std::fprintf(csv, "%.5f,%d,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.2f\n",
                     t, st.stationary ? 1 : 0, st.quaternion.w, st.quaternion.x, st.quaternion.y, st.quaternion.z,
                     st.velocity.x, st.velocity.y, st.velocity.z, st.position.x, st.position.y, st.position.z,
                     st.earthAccel.x, st.earthAccel.y, st.earthAccel.z, st.gyro.x, st.gyro.y, st.gyro.z, loc.currentAhrsGain());
      }
    }
    else if (fr.type == RAW_LOG_TYPE_PRESET || fr.type == RAW_LOG_TYPE_CALIBRATION)
    {
      CalibrationPreset_t preset;
      if (!decodePreset(fr, preset) || noPreset) continue;
      if (preset.accel_valid && !accelPresetOverride)
      {
        loc.setAccelBiasPreset(vec3(preset.accel_bias[0], preset.accel_bias[1], preset.accel_bias[2]));
        std::printf("accel bias preset from recording: [%.4f %.4f %.4f] m/s^2\n", preset.accel_bias[0], preset.accel_bias[1], preset.accel_bias[2]);
      }
      if (preset.gyro_valid && !gyroPresetOverride)
      {
        loc.setGyroBiasPreset(vec3(preset.gyro_bias[0], preset.gyro_bias[1], preset.gyro_bias[2]), true);
        std::printf("gyro bias preset from recording: [%.5f %.5f %.5f] deg/s (calibrated at %.1f C)\n",
                    preset.gyro_bias[0] * 57.2958f, preset.gyro_bias[1] * 57.2958f, preset.gyro_bias[2] * 57.2958f, preset.temperature);
      }
    }
    else if (fr.type == RAW_LOG_TYPE_POSITION)
    {
      PositionData_t dev;
      std::memcpy(&dev, fr.payload.data(), sizeof(dev));
      posCount++;
      // The device frame for timestamp T is produced from the sample with
      // that timestamp, which the host has just processed (frames are in
      // order), so compare with the current host state.
      for (auto it = history.rbegin(); it != history.rend(); ++it)
      {
        if (it->timestamp != dev.timestamp) continue;
        const Vec3 dp = vec3Sub(vec3(dev.position[0], dev.position[1], dev.position[2]), it->position);
        const float d = vec3Norm(dp);
        if (d > maxDevicePosDiff) maxDevicePosDiff = d;
        const float dq = fabsf(dev.quaternion[0] - it->quaternion.w) + fabsf(dev.quaternion[1] - it->quaternion.x) +
                         fabsf(dev.quaternion[2] - it->quaternion.y) + fabsf(dev.quaternion[3] - it->quaternion.z);
        if (dq > maxDeviceQuatDiff) maxDeviceQuatDiff = dq;
        deviceCompared++;
        break;
      }
    }
  }
  if (csv) std::fclose(csv);

  const LocalizerDiagnostics &d = loc.diagnostics();
  const double duration = haveFirst ? (double)(uint32_t)(lastState.timestamp - firstTs) / LOCALIZER_TIMESTAMP_HZ : 0.0;
  std::printf("frames: %zu (imu %u, position %u), unparsable bytes: %zu\n", frames.size(), imuCount, posCount, bad);
  std::printf("phase: %s, running samples: %u, dt anomalies: %u, max dt: %.3f ms, duration: %.1f s\n",
              loc.phase() == Localizer::RUNNING ? "RUNNING" : "CALIBRATING", running, d.dtAnomalies, d.maxDt * 1000.0, duration);
  if (loc.phase() == Localizer::RUNNING)
  {
    const AlignmentResult &a = loc.alignment();
    std::printf("alignment: gyro offset [%.4f %.4f %.4f] deg/s, accel bias [%.4f %.4f %.4f] m/s^2, north %s\n",
                a.gyroOffset.x * 57.2958f, a.gyroOffset.y * 57.2958f, a.gyroOffset.z * 57.2958f,
                a.accelBias.x, a.accelBias.y, a.accelBias.z, a.northValid ? "valid" : "undefined");
    // With the gyro bias removed the horizontal rate is the horizontal earth
    // rate; a large deviation means the gyro preset no longer fits.
    const float earthHorizontal = LOCALIZER_EARTH_RATE * cosf(LOCALIZER_LATITUDE_DEG * LOCALIZER_PI / 180.0f);
    std::printf("horizontal rate used for north: %.4f deg/s (earth rate %.4f deg/s), gyro preset %s\n",
                a.horizontalRate * 57.2958f, earthHorizontal * 57.2958f, a.gyroPresetUsed ? "applied" : "none");
    const Vec3 b = loc.accelBias();
    std::printf("final accel bias [%.4f %.4f %.4f], stationary %.1f %% of samples, max speed %.3f m/s\n",
                b.x, b.y, b.z, running ? 100.0 * stationaryCount / running : 0.0, maxSpeed);
    std::printf("final position [%.4f %.4f %.4f] m (|p| = %.4f), ZUPT corrections: %u\n",
                lastPos.x, lastPos.y, lastPos.z, vec3Norm(lastPos), loc.integrator().correctionCount());
    if (deviceCompared > 0)
    {
      std::printf("device vs host: %u frames compared, max position diff %.6f m, max quaternion diff %.6f\n",
                  deviceCompared, maxDevicePosDiff, maxDeviceQuatDiff);
    }
    if (loc.headingObserverActive())
    {
      const HeadingObserver &o = loc.headingObserver();
      const HeadingObserverStatus &h = o.status();
      const Vec3 preset = loc.gyroBiasPreset();
      std::printf("heading observer: %d rests, correction %+.2f deg applied (%+.2f pending), %u fits, %u rests rejected\n",
                  h.periods, h.yawApplied * 57.2958f, h.yawPending * 57.2958f, h.solves, h.rejectedPeriods);
      std::printf("  gyro bias [%.5f %.5f %.5f] deg/s (preset [%.5f %.5f %.5f], difference [%.5f %.5f %.5f]), fit residual %.5f deg/s, horizontal rate seen %.2f x earth\n",
                  h.bias.x * 57.2958f, h.bias.y * 57.2958f, h.bias.z * 57.2958f,
                  preset.x * 57.2958f, preset.y * 57.2958f, preset.z * 57.2958f,
                  (h.bias.x - preset.x) * 57.2958f, (h.bias.y - preset.y) * 57.2958f, (h.bias.z - preset.z) * 57.2958f,
                  h.residualRms * 57.2958f, h.yawScale);
      for (int i = 0; i < o.periodCount(); i++)
      {
        const HeadingObserver::Period &pd = o.period(i);
        const Vec3 r = vec3Sub(pd.mean, h.bias);
        const Vec3 up = quatRotateEarthToSensor(pd.attitude, vec3(0, 0, 1));
        const Vec3 x = quatRotateSensorToEarth(pd.attitude, vec3(1, 0, 0));
        std::printf("  rest %d: %.1f s, sensor x heading %.1f deg (0 = North, 90 = West), earth rate seen %.2f deg/h (vertical %.2f, expected %.2f / %.2f)\n",
                    i, pd.duration, atan2f(x.y, x.x) * 57.2958f, vec3Norm(r) * 57.2958f * 3600.0f, vec3Dot(r, up) * 57.2958f * 3600.0f,
                    LOCALIZER_EARTH_RATE * 57.2958f * 3600.0f, LOCALIZER_EARTH_RATE * sinf(LOCALIZER_LATITUDE_DEG * LOCALIZER_PI / 180.0f) * 57.2958f * 3600.0f);
      }
    }
  }
  return 0;
}
