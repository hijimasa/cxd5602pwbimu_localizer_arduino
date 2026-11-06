/**
 * @file gyro_bias_fit.h
 * @brief Gyroscope bias estimation from the six-orientation calibration
 *        (header-only, no Arduino dependency; also used by MainCore).
 *
 * At rest the gyroscope measures R^T w_e + b, where w_e is the earth
 * rotation. Its horizontal part depends on the (unknown) heading of each
 * orientation, but its component along gravity is the same everywhere:
 * u . w_e = W sin(latitude), with u the local Up direction. Up is known in
 * the sensor frame from the accelerometer mean of the same orientation (the
 * accelerometer bias from the sphere fit is removed first), so every
 * orientation gives one linear equation in the bias:
 *     u_i . b = u_i . g_i - W sin(lat)
 * Three or more orientations whose Up directions span 3D determine b; the
 * six faces (+/-X, +/-Y, +/-Z up) do so with the latitude term cancelling
 * exactly between the two faces of each axis.
 *
 * Solving W sin(lat) as a fourth unknown gives an independent check that
 * the measurements really contain the earth rotation (estimateVerticalRate).
 */
#ifndef GYRO_BIAS_FIT_H
#define GYRO_BIAS_FIT_H

#include "vec_math.h"
#include "accel_bias_fit.h" // biasFitSolveLinear

#define GYRO_BIAS_FIT_MAX_ORIENTATIONS ACCEL_BIAS_FIT_MAX_ORIENTATIONS

class GyroBiasFit
{
public:
  GyroBiasFit() : count_(0) {}

  void reset() { count_ = 0; }
  int count() const { return count_; }

  /** Add the mean accelerometer and gyroscope readings of one stationary orientation. */
  bool add(Vec3 meanAccel, Vec3 meanGyro)
  {
    if (count_ >= GYRO_BIAS_FIT_MAX_ORIENTATIONS) return false;
    accel_[count_] = meanAccel;
    gyro_[count_] = meanGyro;
    count_++;
    return true;
  }

  /**
   * Solve for the gyro bias.
   * @param accelBias   accelerometer bias (m/s^2) from AccelBiasFit, removed
   *                    before the Up direction is taken
   * @param latitudeDeg latitude of the calibration site
   * @param earthRate   earth rotation rate (rad/s)
   * @param bias        result (rad/s, sensor frame)
   * @param rmsResidual optional: RMS of u_i . (g_i - b) - W sin(lat) (rad/s)
   * @return false if fewer than 3 orientations or if their Up directions do
   *         not span 3D
   */
  bool solve(Vec3 accelBias, float latitudeDeg, float earthRate, Vec3 &bias, float *rmsResidual = 0) const
  {
    if (count_ < 3) return false;
    const float vertical = verticalRate(latitudeDeg, earthRate);
    float ata[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
    float aty[4] = {0, 0, 0, 0};
    for (int i = 0; i < count_; i++)
    {
      Vec3 u = vec3Zero();
      if (!upDirection(i, accelBias, u)) return false;
      const float row[3] = {u.x, u.y, u.z};
      const float y = vec3Dot(u, gyro_[i]) - vertical;
      for (int p = 0; p < 3; p++)
      {
        aty[p] += row[p] * y;
        for (int q = 0; q < 3; q++) ata[p][q] += row[p] * row[q];
      }
    }
    float x[4];
    if (!biasFitSolveLinear(3, ata, aty, x, 0.05f)) return false; // degenerate orientations
    bias = vec3(x[0], x[1], x[2]);
    if (rmsResidual)
    {
      float s = 0.0f;
      for (int i = 0; i < count_; i++)
      {
        Vec3 u = vec3Zero();
        upDirection(i, accelBias, u);
        const float r = vec3Dot(u, vec3Sub(gyro_[i], bias)) - vertical;
        s += r * r;
      }
      *rmsResidual = sqrtf(s / count_);
    }
    return true;
  }

  /**
   * Estimate the vertical component of the earth rotation (rad/s) from the
   * measurements alone, with the bias as nuisance parameter. Should come out
   * near earthRate * sin(latitude); needs orientations with opposite Up
   * directions (the six faces have them).
   */
  bool estimateVerticalRate(Vec3 accelBias, float &rate) const
  {
    if (count_ < 4) return false;
    float ata[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
    float aty[4] = {0, 0, 0, 0};
    for (int i = 0; i < count_; i++)
    {
      Vec3 u = vec3Zero();
      if (!upDirection(i, accelBias, u)) return false;
      const float row[4] = {u.x, u.y, u.z, 1.0f};
      const float y = vec3Dot(u, gyro_[i]);
      for (int p = 0; p < 4; p++)
      {
        aty[p] += row[p] * y;
        for (int q = 0; q < 4; q++) ata[p][q] += row[p] * row[q];
      }
    }
    float x[4];
    if (!biasFitSolveLinear(4, ata, aty, x, 0.05f)) return false;
    rate = x[3];
    return true;
  }

  /** Vertical component of the earth rotation at a latitude (rad/s). */
  static float verticalRate(float latitudeDeg, float earthRate)
  {
    return earthRate * sinf(latitudeDeg * 3.14159265358979f / 180.0f);
  }

private:
  bool upDirection(int i, Vec3 accelBias, Vec3 &u) const
  {
    const Vec3 a = vec3Sub(accel_[i], accelBias);
    const float n = vec3Norm(a);
    if (n < 1e-3f) return false;
    u = vec3Scale(a, 1.0f / n);
    return true;
  }

  Vec3 accel_[GYRO_BIAS_FIT_MAX_ORIENTATIONS];
  Vec3 gyro_[GYRO_BIAS_FIT_MAX_ORIENTATIONS];
  int count_;
};

#endif // GYRO_BIAS_FIT_H
