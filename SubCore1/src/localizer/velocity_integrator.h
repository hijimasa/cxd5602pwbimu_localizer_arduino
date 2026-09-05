/**
 * @file velocity_integrator.h
 * @brief Trapezoidal integration of Earth-frame acceleration into velocity
 *        and position, with ZUPT reset, retroactive ZUPT correction and a
 *        runaway velocity guard.
 *
 * Retroactive correction: when a zero-velocity update is confirmed the
 * velocity should be zero; the remaining velocity is the error accumulated
 * during the motion segment that just ended. The sensor has already been
 * still for the confirmation time t_still, during which the velocity error
 * was constant (position error v_err * t_still); before that, over the
 * remaining T - t_still, the error is modelled as growing linearly
 * (position error v_err * (T - t_still) / 2). To keep the output continuous
 * the correction is spread linearly over ZUPT_CORRECTION_BLEND_TIME after
 * the ZUPT instead of being applied as a step.
 */
#ifndef VELOCITY_INTEGRATOR_H
#define VELOCITY_INTEGRATOR_H

#include <stdint.h>
#include "localizer_config.h"
#include "vec_math.h"

class VelocityIntegrator
{
public:
  VelocityIntegrator();

  void reset();

  /**
   * @param earthAccel Earth-frame acceleration with gravity removed (m/s^2)
   * @param dt         time step (s)
   * @param stationary true to apply a zero-velocity update
   * @param stillTime  seconds the still condition has already held when the
   *                   ZUPT is confirmed (confirmation delay); 0 if unknown
   */
  void update(Vec3 earthAccel, float dt, bool stationary, float stillTime = 0.0f);

  Vec3 velocity() const { return velocity_; }
  Vec3 position() const { return position_; }

  void setRetroactiveCorrection(bool enabled) { retroactive_ = enabled; }
  /** Blend duration in seconds (0 = apply corrections instantly). */
  void setCorrectionBlendTime(float seconds) { blendTime_ = seconds; }

  /** Total position correction decided at the most recent ZUPT (m). */
  Vec3 lastCorrection() const { return lastCorrection_; }
  /** Part of the correction(s) not yet applied to the position (m). */
  Vec3 pendingCorrection() const { return pendingCorrection_; }
  /** Velocity error observed at the most recent ZUPT (m/s). */
  Vec3 lastVelocityError() const { return lastVelocityError_; }
  /** Duration of the motion segment that ended at the most recent ZUPT (s). */
  float lastSegmentDuration() const { return lastSegmentDuration_; }
  uint32_t correctionCount() const { return correctionCount_; }

private:
  Vec3 velocity_;
  Vec3 position_;
  Vec3 oldAccel_;
  Vec3 oldVelocity_;
  bool initialised_;

  bool retroactive_;
  float blendTime_;
  Vec3 pendingCorrection_;
  float blendRemaining_;
  bool wasStationary_;
  float segmentDuration_;   // time since the current motion segment began
  bool segmentClamped_;     // velocity limit hit during this segment
  Vec3 lastCorrection_;
  Vec3 lastVelocityError_;
  float lastSegmentDuration_;
  uint32_t correctionCount_;
};

#endif // VELOCITY_INTEGRATOR_H
