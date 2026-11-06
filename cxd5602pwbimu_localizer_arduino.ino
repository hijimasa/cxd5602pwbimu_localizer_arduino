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
 *   C  start the six-orientation calibration (accelerometer and gyro bias, guided)
 *   Y  confirm the current orientation during the calibration
 *   Q  abort the calibration
 *   P  print the stored calibration preset
 *   X  erase the stored preset and restart the localizer
 *   R  restart the localizer (15 s stationary calibration)
 * The preset is stored in the SPI flash (PRESET_FILE) and loaded at boot.
 */

#ifdef SUBCORE
#error "Core selection is wrong!! Must select MainCore"
#endif

// #define VERBOSE_OUTPUT   // human readable statistics every 5 s
// #define RAW_LOG_OUTPUT   // binary raw IMU + position frames at RAW_LOG_BAUD (for test/replay)

#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/cxd56_cxd5602pwbimu.h>
#include <stdbool.h>
#include <MP.h>
#include <File.h>
#include <Flash.h>

#include "shared_types.h"
#include "localizer_config.h" // gravity, latitude, earth rate (link into SubCore1/src/localizer/)
#include "vec_math.h"
#include "accel_bias_fit.h"
#include "gyro_bias_fit.h"

#define CXD5602PWBIMU_DEVPATH "/dev/imu0"
#define MAX_NFIFO (1)

static cxd5602pwbimu_data_t g_data[MAX_NFIFO];
static int devfd;

// Shared with SubCore1 (address is sent once at start-up)
static ImuRing_t g_imu_ring;

static ProcessingStats_t g_stats;
static bool g_stats_valid = false;

// ---- Calibration preset (flash) and guided six-orientation calibration ---
// "accel_bias ax ay az" (m/s^2), "gyro_bias gx gy gz" (rad/s), "temperature t"
#define PRESET_FILE "calibration.txt"
// accelerometer-only file written by earlier firmware: "ax ay az"
#define LEGACY_PRESET_FILE "accel_bias.txt"
#define CALIB_ORIENTATIONS 6
#define CALIB_SECONDS_PER_ORIENTATION 10
#define CALIB_SAMPLES_PER_ORIENTATION (MESUREMENT_FREQUENCY * CALIB_SECONDS_PER_ORIENTATION)
#define CALIB_MAX_GYRO_DEG_S 1.0f  // stillness check
#define CALIB_MAX_ACCEL_STD 0.05f  // m/s^2
#define RAD_TO_DEG_PER_HOUR (57.2958f * 3600.0f)

static CalibrationPreset_t g_preset; // shared with SubCore1 by pointer: keep static
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
static GyroBiasFit g_calib_gyro_fit;
static double g_calib_sum[3];
static double g_calib_sumsq[3];
static double g_calib_gyro_sum[3];
static double g_calib_temp_sum = 0.0;       // current orientation
static float g_calib_temp_mean_sum = 0.0f;  // sum of the per-orientation means
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
  while (MP.Send(MSG_ID_PRESET, &g_preset, SUBCORE_LOCALIZER) < 0 && retry++ < 100)
  {
    delay(10);
  }
}

static void clearPreset()
{
  memset(&g_preset, 0, sizeof(g_preset));
}

static bool readTextFile(const char *name, char *buf, size_t size)
{
  File f = Flash.open(name, FILE_READ);
  if (!f)
  {
    return false;
  }
  size_t n = 0;
  while (f.available() && n < size - 1)
  {
    buf[n++] = (char)f.read();
  }
  buf[n] = '\0';
  f.close();
  return true;
}

static bool loadPreset()
{
  clearPreset();
  char text[256];
  if (readTextFile(PRESET_FILE, text, sizeof(text)))
  {
    float a[3], g[3], t;
    const char *line = strstr(text, "accel_bias ");
    if (line && sscanf(line, "accel_bias %f %f %f", &a[0], &a[1], &a[2]) == 3)
    {
      for (int i = 0; i < 3; i++) g_preset.accel_bias[i] = a[i];
      g_preset.accel_valid = 1;
    }
    line = strstr(text, "gyro_bias ");
    if (line && sscanf(line, "gyro_bias %f %f %f", &g[0], &g[1], &g[2]) == 3)
    {
      for (int i = 0; i < 3; i++) g_preset.gyro_bias[i] = g[i];
      g_preset.gyro_valid = 1;
    }
    line = strstr(text, "temperature ");
    if (line && sscanf(line, "temperature %f", &t) == 1)
    {
      g_preset.temperature = t;
    }
    return g_preset.accel_valid || g_preset.gyro_valid;
  }
  if (readTextFile(LEGACY_PRESET_FILE, text, sizeof(text)))
  {
    float a[3];
    if (sscanf(text, "%f %f %f", &a[0], &a[1], &a[2]) == 3)
    {
      for (int i = 0; i < 3; i++) g_preset.accel_bias[i] = a[i];
      g_preset.accel_valid = 1;
      return true;
    }
  }
  return false;
}

static bool savePreset()
{
  Flash.remove(PRESET_FILE);
  Flash.remove(LEGACY_PRESET_FILE);
  File f = Flash.open(PRESET_FILE, FILE_WRITE);
  if (!f)
  {
    return false;
  }
  char line[96];
  if (g_preset.accel_valid)
  {
    snprintf(line, sizeof(line), "accel_bias %.6f %.6f %.6f\n", g_preset.accel_bias[0], g_preset.accel_bias[1], g_preset.accel_bias[2]);
    f.print(line);
  }
  if (g_preset.gyro_valid)
  {
    snprintf(line, sizeof(line), "gyro_bias %.8f %.8f %.8f\n", g_preset.gyro_bias[0], g_preset.gyro_bias[1], g_preset.gyro_bias[2]);
    f.print(line);
  }
  snprintf(line, sizeof(line), "temperature %.2f\n", g_preset.temperature);
  f.print(line);
  f.close();
  return true;
}

static void printPreset()
{
  if (!g_preset.accel_valid && !g_preset.gyro_valid)
  {
    Serial.println("# preset: none (send C to calibrate)");
    return;
  }
  Serial.print("# preset:");
  if (g_preset.accel_valid)
  {
    Serial.printf(" accel bias [%.4f %.4f %.4f] m/s^2", g_preset.accel_bias[0], g_preset.accel_bias[1], g_preset.accel_bias[2]);
  }
  if (g_preset.gyro_valid)
  {
    Serial.printf("%s gyro bias [%.5f %.5f %.5f] deg/s", g_preset.accel_valid ? "," : "",
                  g_preset.gyro_bias[0] * 57.2958f, g_preset.gyro_bias[1] * 57.2958f, g_preset.gyro_bias[2] * 57.2958f);
  }
  else
  {
    Serial.print(", gyro bias: none");
  }
  Serial.printf(", calibrated at %.1f C\n", g_preset.temperature);
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
  g_calib_gyro_fit.reset();
  g_calib_temp_mean_sum = 0.0f;
  g_calib_index = 0;
  Serial.printf("# bias calibration (accelerometer and gyroscope): six orientations, %d s each. Output is paused.\n",
                CALIB_SECONDS_PER_ORIENTATION);
  calibPrompt();
}

static void calibBeginMeasure()
{
  for (int i = 0; i < 3; i++)
  {
    g_calib_sum[i] = 0.0;
    g_calib_sumsq[i] = 0.0;
    g_calib_gyro_sum[i] = 0.0;
  }
  g_calib_temp_sum = 0.0;
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
  if (!g_calib_fit.solve(LOCALIZER_GRAVITY, bias, &rms))
  {
    calibAbort("orientations not diverse enough to solve for the bias");
    return;
  }
  // The gyro fit uses the same Up directions (after removing the accel bias)
  // and the vertical earth rate for the configured latitude.
  Vec3 gyroBias = vec3Zero();
  float gyroRms = 0.0f, verticalRate = 0.0f;
  const bool gyroOk = g_calib_gyro_fit.solve(bias, LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE, gyroBias, &gyroRms);
  const bool verticalOk = gyroOk && g_calib_gyro_fit.estimateVerticalRate(bias, verticalRate);

  g_preset.accel_bias[0] = bias.x;
  g_preset.accel_bias[1] = bias.y;
  g_preset.accel_bias[2] = bias.z;
  g_preset.accel_valid = 1;
  g_preset.gyro_bias[0] = gyroBias.x;
  g_preset.gyro_bias[1] = gyroBias.y;
  g_preset.gyro_bias[2] = gyroBias.z;
  g_preset.gyro_valid = gyroOk ? 1 : 0;
  g_preset.temperature = g_calib_temp_mean_sum / CALIB_ORIENTATIONS;
  const bool saved = savePreset();

  Serial.printf("# result: accel bias [%.4f %.4f %.4f] m/s^2 (rms residual %.4f)", bias.x, bias.y, bias.z, rms);
  if (gyroOk)
  {
    Serial.printf(", gyro bias [%.5f %.5f %.5f] deg/s (rms residual %.5f)",
                  gyroBias.x * 57.2958f, gyroBias.y * 57.2958f, gyroBias.z * 57.2958f, gyroRms * 57.2958f);
  }
  else
  {
    Serial.print(", gyro bias not determined");
  }
  Serial.printf(", at %.1f C, %s\n", g_preset.temperature, saved ? "saved to flash" : "FAILED to save to flash");
  if (verticalOk)
  {
    // Sanity check of the earth-rotation model: should match the latitude.
    Serial.printf("# vertical earth rate seen by the gyro: %.1f deg/h (expected %.1f deg/h at %.2f deg latitude)\n",
                  verticalRate * RAD_TO_DEG_PER_HOUR,
                  GyroBiasFit::verticalRate(LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE) * RAD_TO_DEG_PER_HOUR,
                  LOCALIZER_LATITUDE_DEG);
  }
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
  g_calib_gyro_sum[0] += s.gx;
  g_calib_gyro_sum[1] += s.gy;
  g_calib_gyro_sum[2] += s.gz;
  g_calib_temp_sum += s.temp;
  g_calib_count++;
  if (g_calib_count < (uint32_t)CALIB_SAMPLES_PER_ORIENTATION)
  {
    return;
  }
  float mean[3], gyroMean[3], maxStd = 0.0f;
  for (int i = 0; i < 3; i++)
  {
    mean[i] = (float)(g_calib_sum[i] / g_calib_count);
    gyroMean[i] = (float)(g_calib_gyro_sum[i] / g_calib_count);
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
  Serial.printf("# orientation %d: mean accel [%.4f %.4f %.4f] m/s^2, mean rate [%.5f %.5f %.5f] deg/s, %.1f C\n",
                g_calib_index + 1, mean[0], mean[1], mean[2],
                gyroMean[0] * 57.2958f, gyroMean[1] * 57.2958f, gyroMean[2] * 57.2958f,
                (float)(g_calib_temp_sum / g_calib_count));
  g_calib_fit.add(vec3(mean[0], mean[1], mean[2]));
  g_calib_gyro_fit.add(vec3(mean[0], mean[1], mean[2]), vec3(gyroMean[0], gyroMean[1], gyroMean[2]));
  g_calib_temp_mean_sum += (float)(g_calib_temp_sum / g_calib_count);
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
    Flash.remove(PRESET_FILE);
    Flash.remove(LEGACY_PRESET_FILE);
    clearPreset();
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

  // Stored calibration preset (six-orientation calibration)
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
  if (g_preset.accel_valid || g_preset.gyro_valid)
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
  writeRawFrame(RAW_LOG_TYPE_CALIBRATION, &g_preset, sizeof(g_preset));
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
