/**
 * @file accel_bias_fit.h
 * @brief Multi-orientation accelerometer bias estimation (header-only, no
 *        Arduino dependency; also used by MainCore's calibration mode).
 *
 * At rest the accelerometer measures R^T g + b. For several orientations
 * the bias b is the point from which all mean measurements are at distance
 * |g|: minimise sum_i (|a_i - b| - g)^2 by Gauss-Newton. Three or more
 * orientations with sufficiently different gravity directions are needed;
 * the standard procedure uses the six faces (+/-X, +/-Y, +/-Z up).
 */
#ifndef ACCEL_BIAS_FIT_H
#define ACCEL_BIAS_FIT_H

#include "vec_math.h"

#define ACCEL_BIAS_FIT_MAX_ORIENTATIONS 8

/**
 * Solve the n x n system a x = y (n <= 4) by Gaussian elimination with
 * partial pivoting. Fails when a pivot is smaller than minPivot, which for
 * the normal equations of the orientation fits means that the orientations
 * do not span all three axes.
 */
static inline bool biasFitSolveLinear(int n, const float a[4][4], const float y[4], float x[4], float minPivot)
{
  float m[4][5];
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < n; j++) m[i][j] = a[i][j];
    m[i][n] = y[i];
  }
  for (int c = 0; c < n; c++)
  {
    int piv = c;
    for (int r = c + 1; r < n; r++)
      if (fabsf(m[r][c]) > fabsf(m[piv][c])) piv = r;
    if (fabsf(m[piv][c]) < minPivot) return false;
    if (piv != c)
      for (int j = 0; j <= n; j++)
      {
        const float t = m[c][j];
        m[c][j] = m[piv][j];
        m[piv][j] = t;
      }
    for (int r = 0; r < n; r++)
    {
      if (r == c) continue;
      const float f = m[r][c] / m[c][c];
      for (int j = c; j <= n; j++) m[r][j] -= f * m[c][j];
    }
  }
  for (int i = 0; i < n; i++) x[i] = m[i][n] / m[i][i];
  return true;
}

class AccelBiasFit
{
public:
  AccelBiasFit() : count_(0) {}

  void reset() { count_ = 0; }
  int count() const { return count_; }

  /** Add the mean accelerometer reading of one stationary orientation. */
  bool add(Vec3 meanAccel)
  {
    if (count_ >= ACCEL_BIAS_FIT_MAX_ORIENTATIONS) return false;
    means_[count_++] = meanAccel;
    return true;
  }

  /**
   * Solve for the bias.
   * @param gravity   local gravity magnitude (m/s^2)
   * @param bias      result
   * @param rmsResidual optional: RMS of (|a_i - b| - g) over the orientations
   * @return false if fewer than 3 orientations or if they are not diverse
   *         enough to determine all three components
   */
  bool solve(float gravity, Vec3 &bias, float *rmsResidual = 0) const
  {
    if (count_ < 3) return false;
    Vec3 b = vec3Zero();
    for (int iter = 0; iter < 30; iter++)
    {
      float jtj[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
      float jtr[4] = {0, 0, 0, 0};
      for (int i = 0; i < count_; i++)
      {
        const Vec3 d = vec3Sub(means_[i], b);
        const float n = vec3Norm(d);
        if (n < 1e-6f) return false;
        const float r = n - gravity;
        const float j[3] = {-d.x / n, -d.y / n, -d.z / n};
        for (int p = 0; p < 3; p++)
        {
          jtr[p] += j[p] * r;
          for (int q = 0; q < 3; q++) jtj[p][q] += j[p] * j[q];
        }
      }
      float step[4];
      // pivot threshold: two well separated orientations per axis give >= 1
      if (!biasFitSolveLinear(3, jtj, jtr, step, 0.05f)) return false; // degenerate orientations
      b = vec3Sub(b, vec3(step[0], step[1], step[2]));
      if (fabsf(step[0]) + fabsf(step[1]) + fabsf(step[2]) < 1e-7f) break;
    }
    bias = b;
    if (rmsResidual)
    {
      float s = 0.0f;
      for (int i = 0; i < count_; i++)
      {
        const float r = vec3Norm(vec3Sub(means_[i], b)) - gravity;
        s += r * r;
      }
      *rmsResidual = sqrtf(s / count_);
    }
    return true;
  }

private:
  Vec3 means_[ACCEL_BIAS_FIT_MAX_ORIENTATIONS];
  int count_;
};

#endif // ACCEL_BIAS_FIT_H
