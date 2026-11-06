/**
 * @file initial_alignment.h
 * @brief Initial attitude / gyro offset / accel bias estimation from a
 *        stationary period.
 *
 * The mean accelerometer vector gives the Up direction. The component of the
 * mean gyroscope vector perpendicular to Up is the horizontal part of the
 * Earth rotation and therefore points North (gyro-compassing). From these two
 * directions a North-West-Up frame is built, which matches the Fusion NWU
 * convention.
 *
 * The gyro bias is usually larger than the horizontal earth rate, which
 * makes the heading arbitrary. When the bias is known from the
 * six-orientation calibration (gyro_bias_fit.h) it is removed before the
 * North direction is taken. The run-time gyro offset is still the measured
 * mean minus the earth rotation, so a preset that has drifted (temperature,
 * turn-on repeatability) can only degrade the heading, never the attitude
 * at rest.
 */
#ifndef INITIAL_ALIGNMENT_H
#define INITIAL_ALIGNMENT_H

#include <stdint.h>
#include "vec_math.h"

struct AlignmentResult
{
  Quat quaternion;      // sensor relative to Earth (Fusion NWU convention)
  Vec3 gyroOffset;      // rad/s, sensor frame, earth rotation already removed
  Vec3 accelBias;       // m/s^2, sensor frame (only the component along gravity is observable)
  Vec3 meanAccel;       // m/s^2, sensor frame
  Vec3 meanGyro;        // rad/s, sensor frame
  Vec3 earthRateSensor; // rad/s, expected earth rotation in sensor frame
  float horizontalRate; // rad/s, magnitude of the horizontal rate used for North (earthRate*cos(lat) if the bias is fully removed)
  bool northValid;      // false if the horizontal gyro component was too small to define North
  bool gyroPresetUsed;  // North was taken after removing a known gyro bias
};

/** Earth rotation vector in the NWU frame for a given latitude. */
Vec3 earthRateNwu(float latitudeDeg, float earthRate);

/**
 * Compute the alignment from mean sensor readings taken while stationary.
 * @param meanAccel mean accelerometer reading (m/s^2)
 * @param meanGyro  mean gyroscope reading (rad/s)
 * @param gravity   local gravity magnitude (m/s^2)
 * @param latitudeDeg latitude used for the earth-rate model
 * @param earthRate earth rotation rate (rad/s)
 * @param gyroBiasPreset optional known gyro bias (rad/s, sensor frame),
 *        removed from the mean gyro before the North direction is taken
 */
AlignmentResult computeAlignment(Vec3 meanAccel, Vec3 meanGyro,
                                 float gravity, float latitudeDeg, float earthRate,
                                 const Vec3 *gyroBiasPreset = 0);

/** Accumulates samples and computes the alignment once enough are collected. */
class InitialAlignment
{
public:
  explicit InitialAlignment(uint32_t requiredSamples);

  void reset();

  /** Change the number of samples to accumulate (also resets). */
  void setRequiredSamples(uint32_t requiredSamples);

  /** Add one sample. Returns true when the required number has been reached. */
  bool add(Vec3 accel, Vec3 gyro);

  bool isComplete() const { return count_ >= required_; }
  uint32_t sampleCount() const { return count_; }
  uint32_t requiredSamples() const { return required_; }

  Vec3 meanAccel() const;
  Vec3 meanGyro() const;

  AlignmentResult compute(float gravity, float latitudeDeg, float earthRate, const Vec3 *gyroBiasPreset = 0) const;

private:
  uint32_t required_;
  uint32_t count_;
  // double accumulators: 28800 samples of ~10 m/s^2 would lose precision in float
  double accelSum_[3];
  double gyroSum_[3];
};

#endif // INITIAL_ALIGNMENT_H
