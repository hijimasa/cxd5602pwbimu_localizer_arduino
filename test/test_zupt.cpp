#include "test_framework.h"
#include "localizer/zupt_detector.h"
#include <random>

// ---- Reference: original SubCore3 implementation (verbatim port) ----------
namespace ref
{
#define ACCEL_VARIANCE_THRESHOLD ZUPT_ACCEL_VARIANCE_THRESHOLD
#define GYRO_VARIANCE_THRESHOLD ZUPT_GYRO_VARIANCE_THRESHOLD
#define REQUIRED_SAMPLES ZUPT_REQUIRED_SAMPLES
#define MOTION_WINDOW_SIZE ZUPT_WINDOW_SIZE
static int zero_velocity_counter = 0;
static float accel_x_buffer[MOTION_WINDOW_SIZE];
static float accel_y_buffer[MOTION_WINDOW_SIZE];
static float accel_z_buffer[MOTION_WINDOW_SIZE];
static float gyro_x_buffer[MOTION_WINDOW_SIZE];
static float gyro_y_buffer[MOTION_WINDOW_SIZE];
static float gyro_z_buffer[MOTION_WINDOW_SIZE];
static int motion_buffer_idx = 0;
static bool buffer_filled = false;

static void reset()
{
  zero_velocity_counter = 0;
  motion_buffer_idx = 0;
  buffer_filled = false;
  for (int i = 0; i < MOTION_WINDOW_SIZE; i++)
  {
    accel_x_buffer[i] = accel_y_buffer[i] = accel_z_buffer[i] = 0.0f;
    gyro_x_buffer[i] = gyro_y_buffer[i] = gyro_z_buffer[i] = 0.0f;
  }
}

static float calculateVariance(const float *buffer, int size)
{
  float mean = 0.0f;
  for (int i = 0; i < size; i++) mean += buffer[i];
  mean /= size;
  float variance = 0.0f;
  for (int i = 0; i < size; i++)
  {
    float diff = buffer[i] - mean;
    variance += diff * diff;
  }
  return variance / size;
}

static bool detectStationary(float ax, float ay, float az, float gx, float gy, float gz)
{
  accel_x_buffer[motion_buffer_idx] = ax;
  accel_y_buffer[motion_buffer_idx] = ay;
  accel_z_buffer[motion_buffer_idx] = az;
  gyro_x_buffer[motion_buffer_idx] = gx;
  gyro_y_buffer[motion_buffer_idx] = gy;
  gyro_z_buffer[motion_buffer_idx] = gz;
  motion_buffer_idx = (motion_buffer_idx + 1) % MOTION_WINDOW_SIZE;
  if (motion_buffer_idx == 0) buffer_filled = true;
  if (!buffer_filled) return false;
  float total_accel_var = calculateVariance(accel_x_buffer, MOTION_WINDOW_SIZE) +
                          calculateVariance(accel_y_buffer, MOTION_WINDOW_SIZE) +
                          calculateVariance(accel_z_buffer, MOTION_WINDOW_SIZE);
  float total_gyro_var = calculateVariance(gyro_x_buffer, MOTION_WINDOW_SIZE) +
                         calculateVariance(gyro_y_buffer, MOTION_WINDOW_SIZE) +
                         calculateVariance(gyro_z_buffer, MOTION_WINDOW_SIZE);
  if (total_accel_var < ACCEL_VARIANCE_THRESHOLD && total_gyro_var < GYRO_VARIANCE_THRESHOLD)
    zero_velocity_counter++;
  else
    zero_velocity_counter = 0;
  return (zero_velocity_counter > REQUIRED_SAMPLES);
}
} // namespace ref

TEST(zupt_matches_reference_implementation)
{
  std::mt19937 rng(5);
  std::normal_distribution<float> n(0.0f, 1.0f);
  ZuptDetector det;
  ref::reset();
  // alternate quiet and noisy segments so both branches are exercised
  for (int i = 0; i < 20000; i++)
  {
    const bool quiet = ((i / 700) % 2) == 0;
    const float sa = quiet ? 0.005f : 0.2f;
    const float sg = quiet ? 0.0005f : 0.05f;
    const Vec3 a = vec3(0.3f + n(rng) * sa, -0.1f + n(rng) * sa, n(rng) * sa);
    const Vec3 g = vec3(0.01f + n(rng) * sg, n(rng) * sg, -0.02f + n(rng) * sg);
    const bool expect = ref::detectStationary(a.x, a.y, a.z, g.x, g.y, g.z);
    const bool got = det.update(a, g);
    if (expect != got)
    {
      CHECK(expect == got);
      break;
    }
  }
}

TEST(zupt_declares_stationary_after_confirmation_period)
{
  ZuptDetector det;
  int firstStationary = -1;
  for (int i = 0; i < 400; i++)
  {
    if (det.update(vec3(0.5f, 0.5f, 0.5f), vec3(0.1f, 0.1f, 0.1f)) && firstStationary < 0)
    {
      firstStationary = i;
    }
  }
  // the first variance is computed when the window fills (index 15), so the
  // counter strictly exceeds REQUIRED_SAMPLES at index 15 + REQUIRED_SAMPLES
  CHECK(firstStationary == ZUPT_WINDOW_SIZE - 1 + ZUPT_REQUIRED_SAMPLES);
}

TEST(zupt_exits_immediately_on_motion)
{
  ZuptDetector det;
  for (int i = 0; i < 400; i++) det.update(vec3Zero(), vec3Zero());
  CHECK(det.isStationary());
  CHECK(!det.update(vec3(5.0f, 0.0f, 0.0f), vec3Zero()));
  CHECK(!det.isStationary());
}

TEST(zupt_gyro_offset_does_not_matter)
{
  ZuptDetector det;
  bool s = false;
  for (int i = 0; i < 400; i++) s = det.update(vec3Zero(), vec3(1.0f, -2.0f, 0.5f));
  CHECK(s);
}
