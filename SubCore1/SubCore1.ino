/**
 * @file SubCore1.ino
 * @brief Localizer processing core.
 *
 * SubCore1 pulls raw IMU samples from the ring buffer owned by MainCore,
 * runs the complete localization pipeline (see src/localizer/localizer.h)
 * and returns position estimates at OUTPUT_RATE_HZ plus periodic
 * statistics to MainCore.
 *
 * Data never passes through a single shared struct: the IMU stream lives in
 * a ring buffer, so a temporary stall on either core is absorbed instead of
 * silently corrupting or dropping samples. Any sample that is nevertheless
 * lost (SubCore lagging by more than IMU_RING_SIZE - 1 samples) is counted.
 */

#if (SUBCORE != 1)
#error "Core selection is wrong!! Must select SubCore1"
#endif

#include <MP.h>
#include "shared_types.h"
#include "src/localizer/localizer.h"

static_assert(MESUREMENT_FREQUENCY == LOCALIZER_SAMPLE_RATE, "sample rate mismatch");

#define OUTPUT_RING_SIZE 8
#define OUTPUT_INTERVAL (MESUREMENT_FREQUENCY / OUTPUT_RATE_HZ)
#define STATS_INTERVAL_MS 5000
#define RECV_TIMEOUT_MS 100 // wake up even if a notification was lost

static Localizer localizer;
static const ImuRing_t *imu_ring = NULL;
static uint32_t ring_tail = 0;
static uint32_t ring_overruns = 0;

// Output buffers are rings too so that MainCore can still be reading the
// previous entry while the next one is written.
static PositionData_t output_ring[OUTPUT_RING_SIZE];
static uint32_t output_index = 0;
static int output_divider = 0;

static ProcessingStats_t stats_ring[2];
static uint32_t stats_index = 0;
static uint32_t last_stats_time = 0;
static uint32_t stats_samples_start = 0;

// Per-sample processing time (window statistics)
static uint32_t process_us_max = 0;
static uint64_t process_us_sum = 0;
static uint32_t process_count = 0;
static uint32_t batch_max = 0;

static void sendPosition(const LocalizerState &st)
{
  PositionData_t &out = output_ring[output_index % OUTPUT_RING_SIZE];
  output_index++;
  out.timestamp = st.timestamp;
  out.quaternion[0] = st.quaternion.w;
  out.quaternion[1] = st.quaternion.x;
  out.quaternion[2] = st.quaternion.y;
  out.quaternion[3] = st.quaternion.z;
  out.velocity[0] = st.velocity.x;
  out.velocity[1] = st.velocity.y;
  out.velocity[2] = st.velocity.z;
  out.position[0] = st.position.x;
  out.position[1] = st.position.y;
  out.position[2] = st.position.z;
  out.acceleration[0] = st.earthAccel.x;
  out.acceleration[1] = st.earthAccel.y;
  out.acceleration[2] = st.earthAccel.z;
  out.gyro[0] = st.gyro.x;
  out.gyro[1] = st.gyro.y;
  out.gyro[2] = st.gyro.z;
  out.temp = st.temp;
  out.is_stationary = st.stationary;
  MP.Send(MSG_ID_POSITION, &out, 0);
}

static void sendStats(uint32_t now)
{
  ProcessingStats_t &s = stats_ring[stats_index % 2];
  stats_index++;
  const LocalizerDiagnostics &d = localizer.diagnostics();
  s.core_id = SUBCORE_LOCALIZER;
  s.phase = (localizer.phase() == Localizer::RUNNING) ? 1 : 0;
  s.calibration_count = localizer.calibrationProgress();
  s.calibration_required = localizer.calibrationRequired();
  s.samples_processed = d.samplesProcessed;
  s.ring_overruns = ring_overruns;
  s.dt_anomalies = d.dtAnomalies;
  const uint32_t elapsed_ms = now - last_stats_time;
  s.actual_rate_hz = (elapsed_ms > 0) ? (d.samplesProcessed - stats_samples_start) * 1000.0f / elapsed_ms : 0.0f;
  s.max_dt = d.maxDt;
  s.max_process_us = process_us_max;
  s.avg_process_us = (process_count > 0) ? (uint32_t)(process_us_sum / process_count) : 0;
  s.max_batch = batch_max;
  process_us_max = 0;
  process_us_sum = 0;
  process_count = 0;
  batch_max = 0;
  const Vec3 bias = localizer.accelBias();
  s.accel_bias[0] = bias.x;
  s.accel_bias[1] = bias.y;
  s.accel_bias[2] = bias.z;
  const Vec3 offset = localizer.alignment().gyroOffset;
  s.gyro_offset[0] = offset.x;
  s.gyro_offset[1] = offset.y;
  s.gyro_offset[2] = offset.z;
  MP.Send(MSG_ID_STATS, &s, 0);
  stats_samples_start = d.samplesProcessed;
  last_stats_time = now;
}

static void processSample(const ImuRawData_t &raw)
{
  const uint32_t t0 = micros();
  ImuSample s;
  s.timestamp = raw.timestamp;
  s.temp = raw.temp;
  s.accel = vec3(raw.ax, raw.ay, raw.az);
  s.gyro = vec3(raw.gx, raw.gy, raw.gz);

  const bool running = localizer.update(s);

  const uint32_t us = micros() - t0;
  if (us > process_us_max) process_us_max = us;
  process_us_sum += us;
  process_count++;

  if (!running)
  {
    return; // still calibrating
  }

  if (++output_divider >= OUTPUT_INTERVAL)
  {
    output_divider = 0;
    sendPosition(localizer.state());
  }
}

void setup()
{
  MP.begin();
  last_stats_time = millis();
}

static void handleMessage(int8_t msgid, uint32_t msgdata)
{
  if (msgid == MSG_ID_RING_ADDR)
  {
    imu_ring = (const ImuRing_t *)msgdata;
    ring_tail = imu_ring->head; // start with the samples that arrive from now on
  }
  // MSG_ID_IMU_AVAILABLE carries no information beyond "wake up"
}

void loop()
{
  int8_t msgid;
  uint32_t msgdata;

  // Wait for a notification (or the timeout) ...
  MP.RecvTimeout(RECV_TIMEOUT_MS);
  if (MP.Recv(&msgid, &msgdata) > 0)
  {
    handleMessage(msgid, msgdata);
  }
  // ... then consume every pending notification: the inter-core FIFO has
  // only 8 receive buffers and PANICs when they are exhausted, so they
  // must never be left queued while we process the ring.
  MP.RecvTimeout(MP_RECV_POLLING);
  while (MP.Recv(&msgid, &msgdata) > 0)
  {
    handleMessage(msgid, msgdata);
  }

  if (imu_ring != NULL)
  {
    ImuRawData_t raw;
    uint32_t batch = 0;
    // Bounded so that the message queue above is serviced regularly even
    // if the ring is being refilled as fast as it is drained.
    while (batch < IMU_RING_SIZE && imu_ring_pop(imu_ring, &ring_tail, &raw, &ring_overruns) == 1)
    {
      processSample(raw);
      batch++;
    }
    if (batch > batch_max) batch_max = batch;
  }

  const uint32_t now = millis();
  if (now - last_stats_time >= STATS_INTERVAL_MS)
  {
    sendStats(now);
  }
}
