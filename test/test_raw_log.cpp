#include "test_framework.h"
#include "raw_log.h"

static void appendFrame(std::vector<uint8_t> &d, uint8_t type, const void *payload, size_t len)
{
  d.push_back(RAW_LOG_SYNC0);
  d.push_back(RAW_LOG_SYNC1);
  d.push_back(type);
  uint8_t chk = type;
  const uint8_t *p = (const uint8_t *)payload;
  for (size_t i = 0; i < len; i++)
  {
    d.push_back(p[i]);
    chk ^= p[i];
  }
  d.push_back(chk);
}

TEST(raw_log_parses_legacy_and_current_preset_frames)
{
  std::vector<uint8_t> d;
  // legacy accelerometer-only preset: float bias[3]; uint8_t valid; padding
  uint8_t legacy[RAW_LOG_LEGACY_PRESET_SIZE] = {0};
  const float legacyBias[3] = {0.001f, -0.012f, 0.040f};
  std::memcpy(legacy, legacyBias, sizeof(legacyBias));
  legacy[12] = 1;
  appendFrame(d, RAW_LOG_TYPE_PRESET, legacy, sizeof(legacy));

  CalibrationPreset_t current = {};
  current.accel_bias[0] = 0.002f;
  current.gyro_bias[2] = -4.1e-4f;
  current.temperature = 38.5f;
  current.accel_valid = 1;
  current.gyro_valid = 1;
  appendFrame(d, RAW_LOG_TYPE_CALIBRATION, &current, sizeof(current));

  ImuRawData_t imu = {};
  imu.timestamp = 12345;
  imu.az = 9.8f;
  appendFrame(d, RAW_LOG_TYPE_IMU, &imu, sizeof(imu));

  size_t bad = 0;
  const std::vector<Frame> frames = parse(d, bad);
  CHECK(bad == 0);
  CHECK(frames.size() == 3);
  if (frames.size() != 3) return;

  CalibrationPreset_t p;
  CHECK(decodePreset(frames[0], p));
  CHECK(p.accel_valid == 1 && p.gyro_valid == 0);
  CHECK_NEAR(p.accel_bias[1], -0.012f, 1e-9);
  CHECK_NEAR(p.gyro_bias[2], 0.0f, 0.0);

  CHECK(decodePreset(frames[1], p));
  CHECK(p.accel_valid == 1 && p.gyro_valid == 1);
  CHECK_NEAR(p.accel_bias[0], 0.002f, 1e-9);
  CHECK_NEAR(p.gyro_bias[2], -4.1e-4f, 1e-12);
  CHECK_NEAR(p.temperature, 38.5f, 1e-9);

  CHECK(!decodePreset(frames[2], p));
  ImuRawData_t back;
  std::memcpy(&back, frames[2].payload.data(), sizeof(back));
  CHECK(back.timestamp == 12345);
}

TEST(raw_log_resynchronises_after_an_unknown_frame_type)
{
  // A frame type this parser does not know is skipped byte by byte and the
  // following frames are still found.
  std::vector<uint8_t> d;
  const uint8_t junk[5] = {1, 2, 3, 4, 5};
  appendFrame(d, 9, junk, sizeof(junk));
  ImuRawData_t imu = {};
  imu.timestamp = 7;
  appendFrame(d, RAW_LOG_TYPE_IMU, &imu, sizeof(imu));
  size_t bad = 0;
  const std::vector<Frame> frames = parse(d, bad);
  CHECK(frames.size() == 1);
  CHECK(bad == 4 + sizeof(junk));
}
