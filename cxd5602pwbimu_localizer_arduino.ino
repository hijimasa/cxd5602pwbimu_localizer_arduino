/**
 * @file cxd5602pwbimu_localizer_arduino.ino
 * @brief MainCore - IMU acquisition and serial output.
 *
 * MainCore only does I/O:
 *   - reads the CXD5602PWBIMU at MESUREMENT_FREQUENCY and appends every
 *     sample to a ring buffer shared with SubCore1
 *   - prints the position estimates that SubCore1 sends back at OUTPUT_RATE_HZ
 *
 * All processing (calibration, AHRS, ZUPT, integration) runs on SubCore1,
 * see SubCore1/src/localizer. The ring buffer decouples the two cores so
 * that a stall on one side never corrupts or silently drops samples.
 */

#ifdef SUBCORE
#error "Core selection is wrong!! Must select MainCore"
#endif

// #define VERBOSE_OUTPUT   // human readable statistics every 5 s
// #define RAW_LOG_OUTPUT   // binary raw IMU + position frames at RAW_LOG_BAUD (for test/replay)

#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/cxd56_cxd5602pwbimu.h>
#include <stdbool.h>
#include <MP.h>

#include "shared_types.h"

#define CXD5602PWBIMU_DEVPATH "/dev/imu0"
#define MAX_NFIFO (1)

static cxd5602pwbimu_data_t g_data[MAX_NFIFO];
static int devfd;

// Shared with SubCore1 (address is sent once at start-up)
static ImuRing_t g_imu_ring;

static ProcessingStats_t g_stats;
static bool g_stats_valid = false;
static uint32_t g_output_count = 0;
static uint32_t g_last_stats_print = 0;
static uint32_t g_notify_failures = 0;

static int start_sensing(int fd, int rate, int adrange, int gdrange, int nfifos)
{
  cxd5602pwbimu_range_t range;

  ioctl(fd, SNIOC_SSAMPRATE, rate);
  range.accel = adrange;
  range.gyro = gdrange;
  ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  ioctl(fd, SNIOC_SFIFOTHRESH, nfifos);
  ioctl(fd, SNIOC_ENABLE, 1);

  return 0;
}

static int drop_50msdata(int fd, int samprate)
{
  int cnt = samprate / 20;
  cnt = ((cnt + MAX_NFIFO - 1) / MAX_NFIFO) * MAX_NFIFO;
  if (cnt == 0)
    cnt = MAX_NFIFO;

  while (cnt)
  {
    read(fd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
    cnt -= MAX_NFIFO;
  }

  return 0;
}

#ifdef RAW_LOG_OUTPUT
static void writeRawFrame(uint8_t type, const void *payload, size_t len)
{
  static uint8_t frame[2 + 1 + 128 + 1];
  if (len > 128) return;
  frame[0] = RAW_LOG_SYNC0;
  frame[1] = RAW_LOG_SYNC1;
  frame[2] = type;
  uint8_t sum = type;
  const uint8_t *p = (const uint8_t *)payload;
  for (size_t i = 0; i < len; i++)
  {
    frame[3 + i] = p[i];
    sum ^= p[i];
  }
  frame[3 + len] = sum;
  Serial.write(frame, 4 + len);
}
#endif

static void printPosition(const PositionData_t *pos_data)
{
#ifdef RAW_LOG_OUTPUT
  writeRawFrame(RAW_LOG_TYPE_POSITION, pos_data, sizeof(PositionData_t));
  return;
#endif
  // Hex-encoded IEEE754 floats, same field order as before:
  // timestamp, temp, gx, gy, gz, ax, ay, az, qw, qx, qy, qz, vx, vy, vz, px, py, pz
  Serial.printf("%08x,%08x,%08x,%08x,"
                "%08x,%08x,%08x,%08x,"
                "%08x,%08x,%08x,%08x,"
                "%08x,%08x,%08x,"
                "%08x,%08x,%08x\n",
                (unsigned int)pos_data->timestamp,
                *(const unsigned int *)&pos_data->temp,
                *(const unsigned int *)&pos_data->gyro[0],
                *(const unsigned int *)&pos_data->gyro[1],
                *(const unsigned int *)&pos_data->gyro[2],
                *(const unsigned int *)&pos_data->acceleration[0],
                *(const unsigned int *)&pos_data->acceleration[1],
                *(const unsigned int *)&pos_data->acceleration[2],
                *(const unsigned int *)&pos_data->quaternion[0],
                *(const unsigned int *)&pos_data->quaternion[1],
                *(const unsigned int *)&pos_data->quaternion[2],
                *(const unsigned int *)&pos_data->quaternion[3],
                *(const unsigned int *)&pos_data->velocity[0],
                *(const unsigned int *)&pos_data->velocity[1],
                *(const unsigned int *)&pos_data->velocity[2],
                *(const unsigned int *)&pos_data->position[0],
                *(const unsigned int *)&pos_data->position[1],
                *(const unsigned int *)&pos_data->position[2]);
}

void setup()
{
#ifdef RAW_LOG_OUTPUT
  // Stay silent for a moment after boot: the flash writer needs a quiet
  // 115200-baud link right after the DTR reset to catch the boot ROM prompt.
  delay(2000);
  Serial.begin(RAW_LOG_BAUD);
#else
  Serial.begin(115200);
#endif
#ifdef VERBOSE_OUTPUT
  Serial.println("MainCore: starting IMU localizer (MainCore I/O + SubCore1 processing)");
#endif

  imu_ring_init(&g_imu_ring);

  // LED0: calibrating (keep the sensor still), LED1: running
  ledOn(LED0);
  ledOff(LED1);

  // Start the processing core and hand it the ring buffer address.
  MP.begin(SUBCORE_LOCALIZER);
  MP.RecvTimeout(MP_RECV_POLLING);
  int retry = 0;
  while (MP.Send(MSG_ID_RING_ADDR, &g_imu_ring, SUBCORE_LOCALIZER) < 0 && retry++ < 100)
  {
    delay(10);
  }

  // Initialize IMU hardware
  board_cxd5602pwbimu_initialize(5);
  devfd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);

  start_sensing(devfd, MESUREMENT_FREQUENCY, 8, 2000, MAX_NFIFO);
  drop_50msdata(devfd, MESUREMENT_FREQUENCY);

#ifdef VERBOSE_OUTPUT
  Serial.println("MainCore: keep the sensor stationary for calibration (15 s)...");
#endif
  g_last_stats_print = millis();
}

static void handleSubCoreMessages()
{
  int8_t msgid;
  void *msgdata;
  // Non-blocking: drain whatever SubCore1 has sent since the last sample.
  while (MP.Recv(&msgid, &msgdata, SUBCORE_LOCALIZER) > 0)
  {
    if (msgid == MSG_ID_POSITION)
    {
      if (g_output_count == 0)
      {
        ledOff(LED0);
        ledOn(LED1); // first estimate: calibration finished
      }
      printPosition((const PositionData_t *)msgdata);
      g_output_count++;
    }
    else if (msgid == MSG_ID_STATS)
    {
      g_stats = *(const ProcessingStats_t *)msgdata;
      g_stats_valid = true;
#ifdef RAW_LOG_OUTPUT
      writeRawFrame(RAW_LOG_TYPE_STATS, &g_stats, sizeof(ProcessingStats_t));
#endif
    }
  }
}

void loop()
{
  handleSubCoreMessages();

#ifdef VERBOSE_OUTPUT
  const uint32_t now = millis();
  if (now - g_last_stats_print >= 5000)
  {
    Serial.println("\n=== Processing Statistics ===");
    Serial.printf("Output rate: %.2f Hz (expected: %d Hz), notify failures: %lu\n",
                  g_output_count / 5.0f, OUTPUT_RATE_HZ, (unsigned long)g_notify_failures);
    if (g_stats_valid)
    {
      Serial.printf("SubCore1: phase=%s rate=%.1f Hz processed=%lu overruns=%lu dt_anomalies=%lu max_dt=%.2f ms\n",
                    g_stats.phase ? "RUNNING" : "CALIBRATING",
                    g_stats.actual_rate_hz,
                    (unsigned long)g_stats.samples_processed,
                    (unsigned long)g_stats.ring_overruns,
                    (unsigned long)g_stats.dt_anomalies,
                    g_stats.max_dt * 1000.0f);
      Serial.printf("  process time: avg=%lu us max=%lu us, max batch=%lu samples\n",
                    (unsigned long)g_stats.avg_process_us, (unsigned long)g_stats.max_process_us,
                    (unsigned long)g_stats.max_batch);
      Serial.printf("  calibration=%lu/%lu accel_bias=[%.4f %.4f %.4f] gyro_offset=[%.5f %.5f %.5f] deg/s\n",
                    (unsigned long)g_stats.calibration_count, (unsigned long)g_stats.calibration_required,
                    g_stats.accel_bias[0], g_stats.accel_bias[1], g_stats.accel_bias[2],
                    g_stats.gyro_offset[0] * 57.2958f, g_stats.gyro_offset[1] * 57.2958f, g_stats.gyro_offset[2] * 57.2958f);
    }
    else
    {
      Serial.println("SubCore1: no statistics received yet");
    }
    Serial.println("=============================\n");
    g_last_stats_print = now;
    g_output_count = 0;
  }
#endif

  // Read one IMU sample (blocking until available) and publish it.
  const int ret = read(devfd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
  if (ret == (int)(sizeof(g_data[0]) * MAX_NFIFO))
  {
    for (int i = 0; i < MAX_NFIFO; i++)
    {
      ImuRawData_t sample;
      sample.timestamp = g_data[i].timestamp;
      sample.temp = g_data[i].temp;
      sample.ax = g_data[i].ax;
      sample.ay = g_data[i].ay;
      sample.az = g_data[i].az;
      sample.gx = g_data[i].gx;
      sample.gy = g_data[i].gy;
      sample.gz = g_data[i].gz;

      const uint32_t head = imu_ring_push(&g_imu_ring, &sample);
#ifdef RAW_LOG_OUTPUT
      writeRawFrame(RAW_LOG_TYPE_IMU, &sample, sizeof(ImuRawData_t));
#endif
      // The notification only wakes SubCore1; the data is already in the
      // ring. It is sent every IMU_NOTIFY_INTERVAL samples so that the
      // receiver's 8-entry FIFO can never overflow (see shared_types.h).
      if (head % IMU_NOTIFY_INTERVAL == 0)
      {
        if (MP.Send(MSG_ID_IMU_AVAILABLE, head, SUBCORE_LOCALIZER) < 0)
        {
          g_notify_failures++;
        }
      }
    }
  }
}
