/**
 * @file velocity_integrator.h
 * @brief Trapezoidal integration of Earth-frame acceleration into velocity
 *        and position, with ZUPT reset and a runaway velocity guard.
 */
#ifndef VELOCITY_INTEGRATOR_H
#define VELOCITY_INTEGRATOR_H

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
   */
  void update(Vec3 earthAccel, float dt, bool stationary);

  Vec3 velocity() const { return velocity_; }
  Vec3 position() const { return position_; }

private:
  Vec3 velocity_;
  Vec3 position_;
  Vec3 oldAccel_;
  Vec3 oldVelocity_;
  bool initialised_;
};

#endif // VELOCITY_INTEGRATOR_H
