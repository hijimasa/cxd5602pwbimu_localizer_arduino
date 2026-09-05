/**
 * @file imu_simulator.h
 * @brief Synthetic IMU data generator for regression tests.
 *
 * A trajectory is described by a callback that returns, for a time t, the
 * true Earth-frame acceleration, the true attitude (sensor relative to Earth,
 * NWU) and the true body angular rate. The simulator adds gravity, earth
 * rotation, sensor bias and white noise and produces ImuSample values with
 * IMU timestamps (19.2 MHz counter, wrap-around supported).
 */
#ifndef IMU_SIMULATOR_H
#define IMU_SIMULATOR_H

#include <random>
#include <functional>
#include "localizer/localizer.h"

struct TruePose
{
  Vec3 earthAccel; // m/s^2, Earth frame, without gravity
  Vec3 velocity;   // m/s, Earth frame
  Vec3 position;   // m, Earth frame
  Quat attitude;   // sensor relative to Earth
  Vec3 bodyRate;   // rad/s, sensor frame
};

typedef std::function<TruePose(double t)> TrajectoryFn;

struct SimConfig
{
  float gravity = LOCALIZER_GRAVITY;
  float latitudeDeg = LOCALIZER_LATITUDE_DEG;
  float earthRate = LOCALIZER_EARTH_RATE;
  Vec3 accelBias = {0.0f, 0.0f, 0.0f}; // m/s^2, sensor frame
  Vec3 gyroBias = {0.0f, 0.0f, 0.0f};  // rad/s, sensor frame
  float accelNoiseStd = 0.0f;          // m/s^2 per sample
  float gyroNoiseStd = 0.0f;           // rad/s per sample
  uint32_t startTimestamp = 0;
  unsigned seed = 1;
  float temp = 25.0f;
};

class ImuSimulator
{
public:
  explicit ImuSimulator(const SimConfig &cfg) : cfg_(cfg), rng_(cfg.seed), normal_(0.0f, 1.0f)
  {
    ticksPerSample_ = (uint32_t)(LOCALIZER_TIMESTAMP_HZ / LOCALIZER_SAMPLE_RATE + 0.5f);
    omegaNwu_ = earthRateNwu(cfg.latitudeDeg, cfg.earthRate);
  }

  /** Generate the sample for step index k (t = k / fs). */
  ImuSample sample(uint64_t k, const TrajectoryFn &traj, TruePose *truthOut = nullptr)
  {
    const double t = (double)k / LOCALIZER_SAMPLE_RATE;
    const TruePose p = traj(t);
    if (truthOut) *truthOut = p;

    const Vec3 up = vec3(0.0f, 0.0f, cfg_.gravity);
    const Vec3 specificForceEarth = vec3Add(p.earthAccel, up); // accelerometer measures a - g, g = -up
    Vec3 accel = quatRotateEarthToSensor(p.attitude, specificForceEarth);
    Vec3 gyro = vec3Add(p.bodyRate, quatRotateEarthToSensor(p.attitude, omegaNwu_));

    accel = vec3Add(accel, cfg_.accelBias);
    gyro = vec3Add(gyro, cfg_.gyroBias);
    if (cfg_.accelNoiseStd > 0.0f)
    {
      accel = vec3Add(accel, vec3(noise() * cfg_.accelNoiseStd, noise() * cfg_.accelNoiseStd, noise() * cfg_.accelNoiseStd));
    }
    if (cfg_.gyroNoiseStd > 0.0f)
    {
      gyro = vec3Add(gyro, vec3(noise() * cfg_.gyroNoiseStd, noise() * cfg_.gyroNoiseStd, noise() * cfg_.gyroNoiseStd));
    }

    ImuSample s;
    s.timestamp = (uint32_t)(cfg_.startTimestamp + (uint64_t)k * ticksPerSample_); // wraps naturally
    s.temp = cfg_.temp;
    s.accel = accel;
    s.gyro = gyro;
    return s;
  }

private:
  float noise() { return normal_(rng_); }

  SimConfig cfg_;
  std::mt19937 rng_;
  std::normal_distribution<float> normal_;
  uint32_t ticksPerSample_;
  Vec3 omegaNwu_;
};

// ---- Trajectory helpers ----------------------------------------------------

/** Quaternion for a rotation of angle (rad) about a unit axis. */
inline Quat quatFromAxisAngle(Vec3 axis, float angle)
{
  const float h = 0.5f * angle;
  const float s = sinf(h);
  Quat q = {cosf(h), axis.x * s, axis.y * s, axis.z * s};
  return quatNormalise(q);
}

/** Minimum-jerk profile s(tau) in [0,1] and derivatives (per unit tau). */
inline void minimumJerk(double tau, double &s, double &ds, double &dds)
{
  if (tau <= 0.0)
  {
    s = 0.0; ds = 0.0; dds = 0.0; return;
  }
  if (tau >= 1.0)
  {
    s = 1.0; ds = 0.0; dds = 0.0; return;
  }
  const double t2 = tau * tau, t3 = t2 * tau, t4 = t3 * tau, t5 = t4 * tau;
  s = 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
  ds = 30.0 * t2 - 60.0 * t3 + 30.0 * t4;
  dds = 60.0 * tau - 180.0 * t2 + 120.0 * t3;
}

/** Stationary at a fixed attitude. */
inline TrajectoryFn stationaryTrajectory(Quat attitude)
{
  return [attitude](double) {
    TruePose p;
    p.earthAccel = vec3Zero();
    p.velocity = vec3Zero();
    p.position = vec3Zero();
    p.attitude = attitude;
    p.bodyRate = vec3Zero();
    return p;
  };
}

/**
 * Stationary until t0, then a straight minimum-jerk translation of
 * `displacement` (Earth frame) lasting `duration`, then stationary again.
 */
inline TrajectoryFn translationTrajectory(Quat attitude, double t0, double duration, Vec3 displacement)
{
  return [=](double t) {
    double s, ds, dds;
    minimumJerk((t - t0) / duration, s, ds, dds);
    TruePose p;
    p.position = vec3Scale(displacement, (float)s);
    p.velocity = vec3Scale(displacement, (float)(ds / duration));
    p.earthAccel = vec3Scale(displacement, (float)(dds / (duration * duration)));
    p.attitude = attitude;
    p.bodyRate = vec3Zero();
    return p;
  };
}

/**
 * Stationary until t0, then a minimum-jerk rotation of `angle` about the
 * body axis `axis` lasting `duration`, then stationary again.
 */
inline TrajectoryFn rotationTrajectory(Quat attitude0, double t0, double duration, Vec3 axis, float angle)
{
  return [=](double t) {
    double s, ds, dds;
    minimumJerk((t - t0) / duration, s, ds, dds);
    TruePose p;
    p.position = vec3Zero();
    p.velocity = vec3Zero();
    p.earthAccel = vec3Zero();
    // body-axis rotation: q(t) = q0 * q_axis(theta)
    p.attitude = quatMultiply(attitude0, quatFromAxisAngle(axis, angle * (float)s));
    p.bodyRate = vec3Scale(axis, angle * (float)(ds / duration));
    return p;
  };
}

#endif // IMU_SIMULATOR_H
