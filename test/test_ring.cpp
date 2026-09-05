#include "test_framework.h"
#include "shared_types.h"
#include <thread>
#include <atomic>

static ImuRawData_t makeSample(uint32_t i)
{
  ImuRawData_t s;
  s.timestamp = i * 10000u;
  s.temp = 25.0f;
  s.ax = (float)i; s.ay = (float)i + 0.5f; s.az = -(float)i;
  s.gx = 0.1f * i; s.gy = 0.2f * i; s.gz = 0.3f * i;
  return s;
}

static bool sampleEquals(const ImuRawData_t &a, const ImuRawData_t &b)
{
  return a.timestamp == b.timestamp && a.ax == b.ax && a.ay == b.ay && a.az == b.az &&
         a.gx == b.gx && a.gy == b.gy && a.gz == b.gz;
}

TEST(ring_empty_returns_zero)
{
  static ImuRing_t ring;
  imu_ring_init(&ring);
  uint32_t tail = 0, overruns = 0;
  ImuRawData_t out = {};
  CHECK(imu_ring_pop(&ring, &tail, &out, &overruns) == 0);
  CHECK(imu_ring_available(&ring, tail) == 0);
}

TEST(ring_bursty_consumer_loses_nothing_within_capacity)
{
  static ImuRing_t ring;
  imu_ring_init(&ring);
  uint32_t tail = 0, overruns = 0, expected = 0;
  ImuRawData_t out = {};
  uint32_t produced = 0;
  // producer bursts of up to IMU_RING_SIZE-1 samples before the consumer runs
  const int bursts[] = {1, 5, 100, IMU_RING_SIZE - 1, 3, 200, 255, 1};
  for (int b : bursts)
  {
    for (int i = 0; i < b; i++)
    {
      const ImuRawData_t s = makeSample(produced++);
      imu_ring_push(&ring, &s);
    }
    CHECK(imu_ring_available(&ring, tail) == (uint32_t)b);
    while (imu_ring_pop(&ring, &tail, &out, &overruns) == 1)
    {
      CHECK(sampleEquals(out, makeSample(expected)));
      expected++;
    }
  }
  CHECK(expected == produced);
  CHECK(overruns == 0);
}

TEST(ring_overrun_is_counted_and_resynchronises)
{
  static ImuRing_t ring;
  imu_ring_init(&ring);
  uint32_t tail = 0, overruns = 0;
  ImuRawData_t out = {};
  const uint32_t burst = IMU_RING_SIZE + 40; // consumer falls behind by 40 (+1 unsafe slot)
  for (uint32_t i = 0; i < burst; i++)
  {
    const ImuRawData_t s = makeSample(i);
    imu_ring_push(&ring, &s);
  }
  CHECK(imu_ring_pop(&ring, &tail, &out, &overruns) == 1);
  CHECK(overruns == burst - (IMU_RING_SIZE - 1));
  // first delivered sample is the oldest intact one
  CHECK(sampleEquals(out, makeSample(burst - (IMU_RING_SIZE - 1))));
  uint32_t count = 1;
  while (imu_ring_pop(&ring, &tail, &out, &overruns) == 1) count++;
  CHECK(count == IMU_RING_SIZE - 1);
  CHECK(tail == burst);
  CHECK(sampleEquals(out, makeSample(burst - 1)));
}

TEST(ring_head_wraparound_is_safe)
{
  static ImuRing_t ring;
  imu_ring_init(&ring);
  ring.head = 0xFFFFFFF0u; // near the 32-bit wrap
  uint32_t tail = ring.head, overruns = 0;
  ImuRawData_t out = {};
  for (uint32_t i = 0; i < 40; i++)
  {
    const ImuRawData_t s = makeSample(i);
    imu_ring_push(&ring, &s);
  }
  uint32_t i = 0;
  while (imu_ring_pop(&ring, &tail, &out, &overruns) == 1)
  {
    CHECK(sampleEquals(out, makeSample(i)));
    i++;
  }
  CHECK(i == 40);
  CHECK(overruns == 0);
}

TEST(ring_concurrent_producer_consumer)
{
  static ImuRing_t ring;
  imu_ring_init(&ring);
  const uint32_t total = 200000;
  std::atomic<bool> done(false);
  std::thread producer([&] {
    for (uint32_t i = 0; i < total; i++)
    {
      const ImuRawData_t s = makeSample(i);
      imu_ring_push(&ring, &s);
      if ((i & 63) == 0) std::this_thread::yield();
    }
    done = true;
  });
  uint32_t tail = 0, overruns = 0, consumed = 0;
  bool consistent = true;
  ImuRawData_t out = {};
  for (;;)
  {
    const int r = imu_ring_pop(&ring, &tail, &out, &overruns);
    if (r == 1)
    {
      // Every delivered sample must be internally consistent (no torn copy)
      const uint32_t i = out.timestamp / 10000u;
      if (!sampleEquals(out, makeSample(i))) consistent = false;
      consumed++;
    }
    else if (done.load() && imu_ring_available(&ring, tail) == 0)
    {
      break;
    }
  }
  producer.join();
  CHECK(consistent);
  CHECK(consumed + overruns == total);
}
