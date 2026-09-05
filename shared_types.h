/**
 * @file shared_types.h
 * @brief Inter-core communication contract between MainCore and SubCore1.
 *
 * MainCore (IMU acquisition + serial output) and SubCore1 (all processing)
 * exchange data through memory owned by the sender. The IMU stream uses a
 * single-producer / single-consumer ring buffer so that a temporarily slow
 * consumer never loses samples silently: every sample is either processed
 * or counted as an overrun.
 */

#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// Configuration shared by both cores
#define MESUREMENT_FREQUENCY 1920 // Full IMU performance = 1920Hz
#define OUTPUT_RATE_HZ 30

// MainCore wakes SubCore1 once every IMU_NOTIFY_INTERVAL samples. The
// receiving side of the inter-core FIFO has only CONFIG_CXD56_CPUFIFO_NBUFFERS
// (8) buffers and PANICs when they are all in use, so notifications must be
// rare enough that a short stall of the consumer can never leave more than
// 8 of them pending. 8 samples = ~4 ms, i.e. SubCore1 may stall ~32 ms.
#define IMU_NOTIFY_INTERVAL 8

// Message IDs for inter-core communication
#define MSG_ID_RING_ADDR 0x01     // MainCore -> SubCore: address of the ImuRing_t
#define MSG_ID_IMU_AVAILABLE 0x02 // MainCore -> SubCore: new samples in the ring (data = head)
#define MSG_ID_POSITION 0x05      // SubCore -> MainCore: pointer to PositionData_t
#define MSG_ID_STATS 0x12         // SubCore -> MainCore: pointer to ProcessingStats_t

// SubCore IDs
#define SUBCORE_LOCALIZER 1

/**
 * @brief Raw IMU data structure (from sensor)
 */
typedef struct
{
  uint32_t timestamp;
  float temp;
  float ax; // m/s^2
  float ay;
  float az;
  float gx; // rad/s
  float gy;
  float gz;
} ImuRawData_t;

/**
 * @brief Position estimation result (SubCore -> MainCore, OUTPUT_RATE_HZ)
 */
typedef struct
{
  uint32_t timestamp;
  float quaternion[4]; // [w, x, y, z]
  float velocity[3];
  float position[3];
  float acceleration[3]; // Earth frame, gravity removed
  float gyro[3];         // rad/s, offset corrected
  float temp;
  bool is_stationary;
} PositionData_t;

/**
 * @brief Processing statistics (SubCore -> MainCore, every few seconds)
 */
typedef struct
{
  uint8_t core_id;
  uint8_t phase;              // 0 = calibrating, 1 = running
  uint32_t calibration_count; // samples accumulated while calibrating
  uint32_t calibration_required; // samples needed to finish calibration
  uint32_t samples_processed; // running-phase samples since boot
  uint32_t ring_overruns;     // samples lost because the SubCore fell behind
  uint32_t dt_anomalies;      // samples whose dt exceeded the anomaly factor
  float actual_rate_hz;       // samples per second processed in the last window
  float max_dt;               // largest dt seen (s)
  uint32_t max_process_us;    // longest per-sample processing time in the window
  uint32_t avg_process_us;    // mean per-sample processing time in the window
  uint32_t max_batch;         // most samples drained from the ring in one wake-up
  float accel_bias[3];        // current accel bias estimate (m/s^2)
  float gyro_offset[3];       // initial gyro offset estimate (rad/s)
} ProcessingStats_t;

/* -------------------------------------------------------------------------
 * Single-producer / single-consumer ring buffer for the IMU stream.
 *
 * The producer (MainCore) writes entries[head % SIZE] and then increments
 * head. The consumer (SubCore) keeps its own tail. As long as the consumer
 * lags by fewer than IMU_RING_SIZE - 1 samples no data is lost, even if
 * notification messages are dropped. Larger lags are detected and counted.
 * ---------------------------------------------------------------------- */
#define IMU_RING_SIZE 256 // ~133 ms at 1920 Hz

typedef struct
{
  volatile uint32_t head; // number of samples ever written
  ImuRawData_t entries[IMU_RING_SIZE];
} ImuRing_t;

#if defined(__arm__)
#define IMU_RING_BARRIER() __asm__ volatile("dmb" ::: "memory")
#else
#define IMU_RING_BARRIER() __atomic_thread_fence(__ATOMIC_SEQ_CST)
#endif

static inline void imu_ring_init(ImuRing_t *ring)
{
  ring->head = 0;
}

/** Producer side: append one sample. Returns the new head. */
static inline uint32_t imu_ring_push(ImuRing_t *ring, const ImuRawData_t *sample)
{
  const uint32_t h = ring->head;
  ring->entries[h % IMU_RING_SIZE] = *sample;
  IMU_RING_BARRIER(); // data must be visible before head
  ring->head = h + 1;
  return h + 1;
}

/**
 * Consumer side: copy the next unread sample.
 * @param tail     consumer's read position (samples consumed so far); updated
 * @param overruns incremented by the number of samples that were overwritten
 *                 before they could be read
 * @return 1 if a sample was copied, 0 if the ring is empty
 */
static inline int imu_ring_pop(const ImuRing_t *ring, uint32_t *tail,
                               ImuRawData_t *out, uint32_t *overruns)
{
  for (;;)
  {
    uint32_t head = ring->head;
    IMU_RING_BARRIER();
    if (head == *tail)
    {
      return 0;
    }
    // A lag of IMU_RING_SIZE means the producer may be writing the slot we
    // are about to read, so the usable capacity is IMU_RING_SIZE - 1.
    if (head - *tail >= IMU_RING_SIZE)
    {
      // Fell behind: skip to the oldest sample that is still intact
      const uint32_t lost = head - *tail - (IMU_RING_SIZE - 1);
      *overruns += lost;
      *tail += lost;
    }
    *out = ring->entries[*tail % IMU_RING_SIZE];
    IMU_RING_BARRIER(); // re-check that the slot was not overwritten meanwhile
    head = ring->head;
    if (head - *tail >= IMU_RING_SIZE)
    {
      *overruns += 1;
      *tail += 1;
      continue;
    }
    *tail += 1;
    return 1;
  }
}

/** Number of unread samples. */
static inline uint32_t imu_ring_available(const ImuRing_t *ring, uint32_t tail)
{
  return ring->head - tail;
}

#endif // SHARED_TYPES_H
