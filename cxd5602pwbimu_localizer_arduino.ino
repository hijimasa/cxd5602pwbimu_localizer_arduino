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
 *
 * Serial commands (one letter, optional newline, normal build only):
 *   C  start the accelerometer bias calibration (six orientations, guided)
 *   Y  confirm the current orientation during the calibration
 *   Q  abort the calibration
 *   P  print the stored accelerometer bias preset
 *   X  erase the stored preset and restart the localizer
 *   R  restart the localizer (15 s stationary calibration)
 * The preset is stored in the SPI flash (ACCEL_BIAS_FILE) and loaded at boot.
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
#include <File.h>
#include <Flash.h>

#include "shared_types.h"
#include "vec_math.h"
#include "accel_bias_fit.h"
#define LOCALIZER_GRAVITY_MAINCORE 9.7975f // must match LOCALIZER_GRAVITY in localizer_config.h

#define CXD5602PWBIMU_DEVPATH "/dev/imu0"
#define MAX_NFIFO (1)

static cxd5602pwbimu_data_t g_data[MAX_NFIFO];
static int devfd;

// Shared with SubCore1 (address is sent once at start-up)
static ImuRing_t g_imu_ring;

static ProcessingStats_t g_stats;
static bool g_stats_valid = false;

// ---- Accelerometer bias preset (flash) and guided calibration ------------
#define ACCEL_BIAS_FILE "accel_bias.txt"
#define CALIB_ORIENTATIONS 6
#define CALIB_SAMPLES_PER_ORIENTATION (MESUREMENT_FREQUENCY * 3) // 3 s
#define CALIB_MAX_GYRO_DEG_S 1.0f                                // stillness check
#define CALIB_MAX_ACCEL_STD 0.05f                                // m/s^2

static AccelBiasPreset_t g_preset; // shared with SubCore1 by pointer: keep static
static const char *const CALIB_ORIENTATION_NAMES[CALIB_ORIENTATIONS] = {
    "+Z up (board flat, components up)",
    "-Z up (board upside down)",
    "+X up (standing on its -X edge)",
    "-X up (standing on its +X edge)",
    "+Y up (standing on its -Y edge)",
    "-Y up (standing on its +Y edge)"};

enum CalibState
{
  CALIB_IDLE,
  CALIB_WAIT_CONFIRM, // prompt shown, waiting for 'Y'
  CALIB_MEASURING     // accumulating samples
};
static CalibState g_calib_state = CALIB_IDLE;
static int g_calib_index = 0;
static AccelBiasFit g_calib_fit;
static double g_calib_sum[3];
static double g_calib_sumsq[3];
static uint32_t g_calib_count = 0;
static float g_calib_max_gyro = 0.0f;
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

static void markRestart()
{
  g_output_count = 0; // LED0 on until the first estimate arrives again
  ledOn(LED0);
  ledOff(LED1);
}

static void sendPreset()
{
  markRestart();
  int retry = 0;
  while (MP.Send(MSG_ID_ACCEL_BIAS, &g_preset, SUBCORE_LOCALIZER) < 0 && retry++ < 100)
  {
    delay(10);
  }
}

static bool loadPreset()
{
  g_preset.valid = 0;
  g_preset.bias[0] = g_preset.bias[1] = g_preset.bias[2] = 0.0f;
  File f = Flash.open(ACCEL_BIAS_FILE, FILE_READ);
  if (!f)
  {
    return false;
  }
  char line[96];
  size_t n = 0;
  while (f.available() && n < sizeof(line) - 1)
  {
    line[n++] = (char)f.read();
  }
  line[n] = '\0';
  f.close();
  float b[3];
  if (sscanf(line, "%f %f %f", &b[0], &b[1], &b[2]) == 3)
  {
    g_preset.bias[0] = b[0];
    g_preset.bias[1] = b[1];
    g_preset.bias[2] = b[2];
    g_preset.valid = 1;
    return true;
  }
  return false;
}

static bool savePreset()
{
  Flash.remove(ACCEL_BIAS_FILE);
  File f = Flash.open(ACCEL_BIAS_FILE, FILE_WRITE);
  if (!f)
  {
    return false;
  }
  char line[96];
  snprintf(line, sizeof(line), "%.6f %.6f %.6f\n", g_preset.bias[0], g_preset.bias[1], g_preset.bias[2]);
  f.print(line);
  f.close();
  return true;
}

static void printPreset()
{
  if (g_preset.valid)
  {
    Serial.printf("# accel bias preset: [%.4f %.4f %.4f] m/s^2\n", g_preset.bias[0], g_preset.bias[1], g_preset.bias[2]);
  }
  else
  {
    Serial.println("# accel bias preset: none (send C to calibrate)");
  }
}

static void calibPrompt()
{
  Serial.printf("# calibration %d/%d: place the sensor %s, keep it still and send Y (Q to abort)\n",
                g_calib_index + 1, CALIB_ORIENTATIONS, CALIB_ORIENTATION_NAMES[g_calib_index]);
  g_calib_state = CALIB_WAIT_CONFIRM;
}

static void calibStart()
{
  g_calib_fit.reset();
  g_calib_index = 0;
  Serial.println("# accelerometer bias calibration: six orientations, 3 s each. Output is paused.");
  calibPrompt();
}

static void calibBeginMeasure()
{
  for (int i = 0; i < 3; i++)
  {
    g_calib_sum[i] = 0.0;
    g_calib_sumsq[i] = 0.0;
  }
  g_calib_count = 0;
  g_calib_max_gyro = 0.0f;
  g_calib_state = CALIB_MEASURING;
  Serial.println("# measuring...");
}

static void calibAbort(const char *why)
{
  g_calib_state = CALIB_IDLE;
  Serial.printf("# calibration aborted: %s\n", why);
}

static void calibFinish()
{
  Vec3 bias;
  float rms = 0.0f;
  if (!g_calib_fit.solve(LOCALIZER_GRAVITY_MAINCORE, bias, &rms))
  {
    calibAbort("orientations not diverse enough to solve for the bias");
    return;
  }
  g_preset.bias[0] = bias.x;
  g_preset.bias[1] = bias.y;
  g_preset.bias[2] = bias.z;
  g_preset.valid = 1;
  const bool saved = savePreset();
  Serial.printf("# result: bias [%.4f %.4f %.4f] m/s^2, rms residual %.4f m/s^2, %s\n",
                bias.x, bias.y, bias.z, rms, saved ? "saved to flash" : "FAILED to save to flash");
  sendPreset(); // SubCore1 restarts its 15 s stationary calibration
  Serial.println("# localizer restarting: keep the sensor still for 15 s");
  g_calib_state = CALIB_IDLE;
}

static void calibFeedSample(const ImuRawData_t &s)
{
  const float gyro = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz) * 57.2958f;
  if (gyro > g_calib_max_gyro) g_calib_max_gyro = gyro;
  g_calib_sum[0] += s.ax;
  g_calib_sum[1] += s.ay;
  g_calib_sum[2] += s.az;
  g_calib_sumsq[0] += (double)s.ax * s.ax;
  g_calib_sumsq[1] += (double)s.ay * s.ay;
  g_calib_sumsq[2] += (double)s.az * s.az;
  g_calib_count++;
  if (g_calib_count < (uint32_t)CALIB_SAMPLES_PER_ORIENTATION)
  {
    return;
  }
  float mean[3], maxStd = 0.0f;
  for (int i = 0; i < 3; i++)
  {
    mean[i] = (float)(g_calib_sum[i] / g_calib_count);
    const float var = (float)(g_calib_sumsq[i] / g_calib_count) - mean[i] * mean[i];
    const float sd = var > 0.0f ? sqrtf(var) : 0.0f;
    if (sd > maxStd) maxStd = sd;
  }
  if (g_calib_max_gyro > CALIB_MAX_GYRO_DEG_S || maxStd > CALIB_MAX_ACCEL_STD)
  {
    Serial.printf("# the sensor moved (max rate %.2f deg/s, accel std %.3f): repeat this orientation\n", g_calib_max_gyro, maxStd);
    calibPrompt();
    return;
  }
  Serial.printf("# orientation %d: mean accel [%.4f %.4f %.4f] m/s^2\n", g_calib_index + 1, mean[0], mean[1], mean[2]);
  g_calib_fit.add(vec3(mean[0], mean[1], mean[2]));
  g_calib_index++;
  if (g_calib_index >= CALIB_ORIENTATIONS)
  {
    calibFinish();
  }
  else
  {
    calibPrompt();
  }
}

static void handleSerialCommand(char c)
{
  switch (c)
  {
  case 'C':
  case 'c':
    if (g_calib_state == CALIB_IDLE) calibStart();
    break;
  case 'Y':
  case 'y':
    if (g_calib_state == CALIB_WAIT_CONFIRM) calibBeginMeasure();
    break;
  case 'Q':
  case 'q':
    if (g_calib_state != CALIB_IDLE) calibAbort("by user");
    break;
  case 'P':
  case 'p':
    printPreset();
    break;
  case 'X':
  case 'x':
    Flash.remove(ACCEL_BIAS_FILE);
    g_preset.valid = 0;
    g_preset.bias[0] = g_preset.bias[1] = g_preset.bias[2] = 0.0f;
    sendPreset();
    Serial.println("# preset erased, localizer restarting: keep the sensor still for 15 s");
    break;
  case 'R':
  case 'r':
  {
    uint32_t dummy = 0;
    MP.Send(MSG_ID_RESET, dummy, SUBCORE_LOCALIZER);
    markRestart();
    Serial.println("# localizer restarting: keep the sensor still for 15 s");
    break;
  }
  default:
    break; // ignore newlines and unknown characters
  }
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

  // Stored accelerometer bias preset (multi-orientation calibration)
  Flash.begin();
  loadPreset();

  // Start the processing core and hand it the ring buffer address.
  MP.begin(SUBCORE_LOCALIZER);
  MP.RecvTimeout(MP_RECV_POLLING);
  int retry = 0;
  while (MP.Send(MSG_ID_RING_ADDR, &g_imu_ring, SUBCORE_LOCALIZER) < 0 && retry++ < 100)
  {
    delay(10);
  }
  if (g_preset.valid)
  {
    sendPreset();
  }

  // Initialize IMU hardware
  board_cxd5602pwbimu_initialize(5);
  devfd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);

  start_sensing(devfd, MESUREMENT_FREQUENCY, 8, 2000, MAX_NFIFO);
  drop_50msdata(devfd, MESUREMENT_FREQUENCY);

#ifdef VERBOSE_OUTPUT
  Serial.println("MainCore: keep the sensor stationary for calibration (15 s)...");
  printPreset();
#endif
#ifdef RAW_LOG_OUTPUT
  writeRawFrame(RAW_LOG_TYPE_PRESET, &g_preset, sizeof(g_preset));
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
      if (g_calib_state == CALIB_IDLE)
      {
        printPosition((const PositionData_t *)msgdata);
      }
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
#ifndef RAW_LOG_OUTPUT
  while (Serial.available() > 0)
  {
    handleSerialCommand((char)Serial.read());
  }
#endif
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

      if (g_calib_state == CALIB_MEASURING)
      {
        calibFeedSample(sample);
      }
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
