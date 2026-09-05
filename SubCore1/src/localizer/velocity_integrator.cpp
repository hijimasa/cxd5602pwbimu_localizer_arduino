#include "velocity_integrator.h"

VelocityIntegrator::VelocityIntegrator()
{
  reset();
}

void VelocityIntegrator::reset()
{
  velocity_ = vec3Zero();
  position_ = vec3Zero();
  oldAccel_ = vec3Zero();
  oldVelocity_ = vec3Zero();
  initialised_ = false;
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
    velocity_ = vec3Zero();
  }
  else
  {
    const float speed = vec3Norm(velocity_);
    if (speed > LOCALIZER_MAX_VELOCITY)
    {
      velocity_ = vec3Scale(velocity_, LOCALIZER_MAX_VELOCITY / speed);
    }
  }

  oldAccel_ = earthAccel;
  oldVelocity_ = velocity_;
}
