/**
 * @file localizer.h
 * @brief Complete per-sample IMU localization pipeline (no Arduino dependency).
 *
 * Phases:
 *  1. CALIBRATING: the sensor must be stationary. Mean accel/gyro are
 *     accumulated and used for initial attitude, gyro offset and accel bias.
 *  2. RUNNING: Gaussian pre-filter -> earth-rate compensation -> Fusion AHRS
 *     -> gravity removal & bias correction -> ZUPT detection -> bias learning
 *     -> trapezoidal velocity/position integration.
 */
#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <stdint.h>
#include "localizer_config.h"
#include "vec_math.h"
#include "initial_alignment.h"
#include "gaussian_filter.h"
#include "zupt_detector.h"
#include "velocity_integrator.h"

extern "C"
{
#include "../Fusion/Fusion.h"
}

struct ImuSample
{
  uint32_t timestamp; // IMU counter ticks (LOCALIZER_TIMESTAMP_HZ)
  float temp;
  Vec3 accel; // m/s^2
  Vec3 gyro;  // rad/s
};

struct LocalizerState
{
  uint32_t timestamp;
  Quat quaternion;  // sensor relative to Earth (NWU)
  Vec3 velocity;    // m/s, Earth frame
  Vec3 position;    // m, Earth frame
  Vec3 earthAccel;  // m/s^2, Earth frame, gravity removed, bias corrected
  Vec3 sensorAccel; // m/s^2, sensor frame, gravity removed, bias corrected
  Vec3 gyro;        // rad/s, sensor frame, offset and earth-rate corrected
  float dt;         // s
  float temp;
  bool stationary;
};

struct LocalizerDiagnostics
{
  uint32_t samplesProcessed; // samples processed in RUNNING phase
  uint32_t dtAnomalies;      // samples whose dt exceeded the anomaly factor
  float lastDt;
  float maxDt;
};

class Localizer
{
public:
  enum Phase
  {
    CALIBRATING,
    RUNNING
  };

  Localizer();

  /** Return to CALIBRATING with all state cleared. */
  void reset();

  /**
   * Process one sample.
   * @return true when a new state is available (RUNNING phase only).
   */
  bool update(const ImuSample &sample);

  Phase phase() const { return phase_; }
  const LocalizerState &state() const { return state_; }
  const AlignmentResult &alignment() const { return alignment_; }
  const LocalizerDiagnostics &diagnostics() const { return diag_; }
  Vec3 accelBias() const { return accelBias_; }

  /** Current run-time gyro offset estimate (rad/s), tracked by FusionOffset. */
  Vec3 gyroOffsetEstimate() const;
  uint32_t calibrationProgress() const { return alignmentAccumulator_.sampleCount(); }
  uint32_t calibrationRequired() const { return alignmentAccumulator_.requiredSamples(); }

  /**
   * Set the stationary calibration length (default LOCALIZER_CALIBRATION_SECONDS).
   * Only meaningful while still CALIBRATING; the accumulator is reset.
   */
  void setCalibrationSeconds(float seconds)
  {
    alignmentAccumulator_.setRequiredSamples((uint32_t)(seconds * LOCALIZER_SAMPLE_RATE + 0.5f));
  }

  /**
   * Accelerometer bias known from a multi-orientation calibration
   * (accel_bias_fit.h), subtracted from every raw sample before anything
   * else. Components perpendicular to gravity cannot be learned at run time,
   * so this is the only way to remove them.
   */
  void setAccelBiasPreset(Vec3 bias) { accelBiasPreset_ = bias; }
  Vec3 accelBiasPreset() const { return accelBiasPreset_; }

  /** Enable/disable earth rotation compensation (enabled by default). */
  void setEarthRateCompensation(bool enabled) { earthRateCompensation_ = enabled; }

  /** Enable/disable accel bias learning while stationary (enabled by default). */
  void setBiasLearning(bool enabled) { biasLearning_ = enabled; }

  /** Enable/disable AHRS gain scheduling from the ZUPT state (enabled by default). */
  void setGainScheduling(bool enabled) { gainScheduling_ = enabled; }

  /** AHRS gain currently applied. */
  float currentAhrsGain() const { return currentGain_; }

  /** Enable/disable the retroactive ZUPT position correction (enabled by default). */
  void setRetroactiveCorrection(bool enabled) { integrator_.setRetroactiveCorrection(enabled); }
  /** Blend duration of the retroactive correction (0 = instant). */
  void setCorrectionBlendTime(float seconds) { integrator_.setCorrectionBlendTime(seconds); }

  /** Integrator access for diagnostics (last ZUPT correction etc.). */
  const VelocityIntegrator &integrator() const { return integrator_; }

private:
  void startRunning(const ImuSample &sample);
  float computeDt(uint32_t timestamp);
  void processRunning(const ImuSample &sample);
  void applyAhrsGain(float gain);

  Phase phase_;
  InitialAlignment alignmentAccumulator_;
  AlignmentResult alignment_;

  CausalGaussianFilter accelFilter_[3];
  CausalGaussianFilter gyroFilter_[3];
  FusionAhrs ahrs_;
  FusionOffset offset_;
  ZuptDetector zupt_;
  VelocityIntegrator integrator_;

  Vec3 accelBias_;
  Vec3 accelBiasPreset_;
  Vec3 earthRateNwu_;
  bool earthRateCompensation_;
  bool biasLearning_;
  bool gainScheduling_;
  float currentGain_;
  uint32_t ambiguousCounter_; // samples in "low variance, large mean" state

  uint32_t lastTimestamp_;
  bool haveLastTimestamp_;

  LocalizerState state_;
  LocalizerDiagnostics diag_;
};

#endif // LOCALIZER_H
