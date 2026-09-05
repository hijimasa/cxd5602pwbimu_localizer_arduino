/**
 * @file zupt_detector.h
 * @brief Variance based zero-velocity (stationary) detector.
 *
 * The sensor is declared stationary when the summed per-axis variance of the
 * acceleration and of the angular rate over a short window stay below their
 * thresholds for ZUPT_REQUIRED_SAMPLES consecutive samples. Leaving the
 * stationary state is immediate.
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
   * @param accel acceleration (any frame, gravity removed) m/s^2
   * @param gyro  angular rate (rad/s); an offset does not affect the variance
   * @return true if stationary
   */
  bool update(Vec3 accel, Vec3 gyro);

  bool isStationary() const { return stationary_; }
  float lastAccelVariance() const { return accelVariance_; }
  float lastGyroVariance() const { return gyroVariance_; }

private:
  static float variance(const float *buffer, int size);

  float accelBuffer_[3][ZUPT_WINDOW_SIZE];
  float gyroBuffer_[3][ZUPT_WINDOW_SIZE];
  int index_;
  bool filled_;
  int stillCounter_;
  bool stationary_;
  float accelVariance_;
  float gyroVariance_;
};

#endif // ZUPT_DETECTOR_H
