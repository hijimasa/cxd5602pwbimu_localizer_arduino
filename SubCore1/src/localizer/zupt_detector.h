/**
 * @file zupt_detector.h
 * @brief Zero-velocity (stationary) detector.
 *
 * Over a short window the detector computes, per axis, the mean and the
 * variance of the gravity-removed acceleration and of the offset-corrected
 * angular rate. The sensor is "still" when
 *   - the summed variances are below their thresholds (no vibration/jerk) and
 *   - the magnitudes of the mean vectors are below their thresholds
 *     (no sustained acceleration or rotation).
 * It is declared stationary when "still" holds for ZUPT_REQUIRED_SAMPLES
 * consecutive samples, or for ZUPT_SLOW_CONFIRM_SAMPLES when the estimated
 * speed is above ZUPT_SLOW_CONFIRM_SPEED (a coasting phase must not be
 * mistaken for a stop); leaving the stationary state is immediate.
 *
 * The variance-only condition is exposed separately (isLowVariance) because
 * it is robust to attitude/bias errors and is used for AHRS gain scheduling.
 */
#ifndef ZUPT_DETECTOR_H
#define ZUPT_DETECTOR_H

#include "localizer_config.h"
#include "vec_math.h"

class ZuptDetector
{
public:
  ZuptDetector();

  void reset();

  /**
   * @param accel gravity-removed, bias-corrected acceleration (m/s^2)
   * @param gyro  offset-corrected angular rate (rad/s)
   * @param speed current estimated speed (m/s), selects the confirmation time
   * @return true if stationary
   */
  bool update(Vec3 accel, Vec3 gyro, float speed = 0.0f);

  bool isStationary() const { return stationary_; }
  /** Consecutive samples for which the still condition has held (0 if moving). */
  int stillSamples() const { return stillCounter_; }
  /** Variance condition alone (false while the window is not yet filled). */
  bool isLowVariance() const { return lowVariance_; }
  /** Mean-magnitude condition alone (false while the window is not yet filled). */
  bool isLowMean() const { return lowMean_; }
  float lastAccelVariance() const { return accelVariance_; }
  float lastGyroVariance() const { return gyroVariance_; }
  float lastAccelMeanNorm() const { return accelMeanNorm_; }
  float lastGyroMeanNorm() const { return gyroMeanNorm_; }

private:
  static void meanAndVariance(const float *buffer, int size, float &mean, float &variance);

  float accelBuffer_[3][ZUPT_WINDOW_SIZE];
  float gyroBuffer_[3][ZUPT_WINDOW_SIZE];
  int index_;
  bool filled_;
  int stillCounter_;
  bool stationary_;
  bool lowVariance_;
  bool lowMean_;
  float accelVariance_;
  float gyroVariance_;
  float accelMeanNorm_;
  float gyroMeanNorm_;
};

#endif // ZUPT_DETECTOR_H
