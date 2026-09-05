#include "velocity_integrator.h"

VelocityIntegrator::VelocityIntegrator()
{
  retroactive_ = ZUPT_RETROACTIVE_CORRECTION != 0;
  blendTime_ = ZUPT_CORRECTION_BLEND_TIME;
  reset();
}

void VelocityIntegrator::reset()
{
  velocity_ = vec3Zero();
  position_ = vec3Zero();
  oldAccel_ = vec3Zero();
  oldVelocity_ = vec3Zero();
  initialised_ = false;
  pendingCorrection_ = vec3Zero();
  blendRemaining_ = 0.0f;
  wasStationary_ = true; // the run starts at rest: no segment is open
  segmentDuration_ = 0.0f;
  segmentClamped_ = false;
  lastCorrection_ = vec3Zero();
  lastVelocityError_ = vec3Zero();
  lastSegmentDuration_ = 0.0f;
  correctionCount_ = 0;
}

void VelocityIntegrator::update(Vec3 earthAccel, float dt, bool stationary)
{
  if (!initialised_)
  {
    oldAccel_ = earthAccel;
    initialised_ = true;
  }

  velocity_ = vec3Add(velocity_, vec3Scale(vec3Add(earthAccel, oldAccel_), 0.5f * dt));
  position_ = vec3Add(position_, vec3Scale(vec3Add(velocity_, oldVelocity_), 0.5f * dt));

  if (stationary)
  {
    if (!wasStationary_)
    {
      // End of a motion segment: the remaining velocity is the error.
      lastVelocityError_ = velocity_;
      lastSegmentDuration_ = segmentDuration_;
      lastCorrection_ = vec3Zero();
      if (retroactive_ && !segmentClamped_ && segmentDuration_ > 0.0f &&
          segmentDuration_ <= ZUPT_MAX_CORRECTION_SEGMENT)
      {
        lastCorrection_ = vec3Scale(velocity_, -0.5f * segmentDuration_);
        correctionCount_++;
        if (blendTime_ > 0.0f)
        {
          // queue it; any remainder of a previous correction is merged
          pendingCorrection_ = vec3Add(pendingCorrection_, lastCorrection_);
          blendRemaining_ = blendTime_;
        }
        else
        {
          position_ = vec3Add(position_, lastCorrection_);
        }
      }
    }
    velocity_ = vec3Zero();
    segmentDuration_ = 0.0f;
    segmentClamped_ = false;
  }
  else
  {
    segmentDuration_ += dt;
    const float speed = vec3Norm(velocity_);
    if (speed > LOCALIZER_MAX_VELOCITY)
    {
      velocity_ = vec3Scale(velocity_, LOCALIZER_MAX_VELOCITY / speed);
      segmentClamped_ = true;
    }
  }
  wasStationary_ = stationary;

  // Apply the pending correction gradually (independent of the ZUPT state)
  if (blendRemaining_ > 0.0f)
  {
    if (dt >= blendRemaining_)
    {
      position_ = vec3Add(position_, pendingCorrection_);
      pendingCorrection_ = vec3Zero();
      blendRemaining_ = 0.0f;
    }
    else
    {
      const Vec3 step = vec3Scale(pendingCorrection_, dt / blendRemaining_);
      position_ = vec3Add(position_, step);
      pendingCorrection_ = vec3Sub(pendingCorrection_, step);
      blendRemaining_ -= dt;
    }
  }

  oldAccel_ = earthAccel;
  oldVelocity_ = velocity_;
}
