/**
 * @file vec_math.h
 * @brief Minimal 3-vector and quaternion helpers (no Arduino dependency).
 *
 * Quaternion convention follows the Fusion library: q describes the sensor
 * relative to the Earth, i.e. v_earth = R(q) * v_sensor.
 */
#ifndef VEC_MATH_H
#define VEC_MATH_H

#include <math.h>

struct Vec3
{
  float x, y, z;
};

struct Quat
{
  float w, x, y, z;
};

static inline Vec3 vec3(float x, float y, float z)
{
  Vec3 v = {x, y, z};
  return v;
}

static inline Vec3 vec3Zero() { return vec3(0.0f, 0.0f, 0.0f); }

static inline Vec3 vec3Add(Vec3 a, Vec3 b) { return vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline Vec3 vec3Sub(Vec3 a, Vec3 b) { return vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline Vec3 vec3Scale(Vec3 a, float s) { return vec3(a.x * s, a.y * s, a.z * s); }
static inline float vec3Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

static inline Vec3 vec3Cross(Vec3 a, Vec3 b)
{
  return vec3(a.y * b.z - a.z * b.y,
              a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x);
}

static inline float vec3Norm(Vec3 a) { return sqrtf(vec3Dot(a, a)); }

static inline Vec3 vec3Normalise(Vec3 a)
{
  float n = vec3Norm(a);
  if (n <= 0.0f)
  {
    return vec3Zero();
  }
  return vec3Scale(a, 1.0f / n);
}

static inline Quat quatIdentity()
{
  Quat q = {1.0f, 0.0f, 0.0f, 0.0f};
  return q;
}

static inline Quat quatConjugate(Quat q)
{
  Quat r = {q.w, -q.x, -q.y, -q.z};
  return r;
}

static inline Quat quatNormalise(Quat q)
{
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n <= 0.0f)
  {
    return quatIdentity();
  }
  Quat r = {q.w / n, q.x / n, q.y / n, q.z / n};
  return r;
}

/** Hamilton product a * b. */
static inline Quat quatMultiply(Quat a, Quat b)
{
  Quat r = {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
  return r;
}

/** Rotate a sensor-frame vector into the Earth frame: R(q) * v. */
static inline Vec3 quatRotateSensorToEarth(Quat q, Vec3 v)
{
  const float r00 = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  const float r01 = 2.0f * (q.x * q.y - q.w * q.z);
  const float r02 = 2.0f * (q.x * q.z + q.w * q.y);
  const float r10 = 2.0f * (q.x * q.y + q.w * q.z);
  const float r11 = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);
  const float r12 = 2.0f * (q.y * q.z - q.w * q.x);
  const float r20 = 2.0f * (q.x * q.z - q.w * q.y);
  const float r21 = 2.0f * (q.y * q.z + q.w * q.x);
  const float r22 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  return vec3(r00 * v.x + r01 * v.y + r02 * v.z,
              r10 * v.x + r11 * v.y + r12 * v.z,
              r20 * v.x + r21 * v.y + r22 * v.z);
}

/** Rotate an Earth-frame vector into the sensor frame: R(q)^T * v. */
static inline Vec3 quatRotateEarthToSensor(Quat q, Vec3 v)
{
  return quatRotateSensorToEarth(quatConjugate(q), v);
}

/**
 * Build the quaternion (sensor relative to Earth, Fusion convention) from
 * the three Earth axes expressed in the sensor frame. The axes must form a
 * right-handed orthonormal triad.
 *
 * With rows (north, west, up) the matrix M satisfies v_earth = M * v_sensor,
 * so this is the standard rotation-matrix-to-quaternion conversion of M.
 */
static inline Quat quatFromEarthAxesInSensorFrame(Vec3 north, Vec3 west, Vec3 up)
{
  const float m00 = north.x, m01 = north.y, m02 = north.z;
  const float m10 = west.x, m11 = west.y, m12 = west.z;
  const float m20 = up.x, m21 = up.y, m22 = up.z;
  const float trace = m00 + m11 + m22;
  Quat q;
  if (trace > 0.0f)
  {
    const float s = sqrtf(trace + 1.0f) * 2.0f;
    q.w = 0.25f * s;
    q.x = (m21 - m12) / s;
    q.y = (m02 - m20) / s;
    q.z = (m10 - m01) / s;
  }
  else if (m00 > m11 && m00 > m22)
  {
    const float s = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
    q.w = (m21 - m12) / s;
    q.x = 0.25f * s;
    q.y = (m01 + m10) / s;
    q.z = (m02 + m20) / s;
  }
  else if (m11 > m22)
  {
    const float s = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
    q.w = (m02 - m20) / s;
    q.x = (m01 + m10) / s;
    q.y = 0.25f * s;
    q.z = (m12 + m21) / s;
  }
  else
  {
    const float s = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
    q.w = (m10 - m01) / s;
    q.x = (m02 + m20) / s;
    q.y = (m12 + m21) / s;
    q.z = 0.25f * s;
  }
  return quatNormalise(q);
}

/** Angle (rad) between two unit vectors. */
static inline float vec3AngleBetween(Vec3 a, Vec3 b)
{
  // atan2 form keeps full precision for small angles (acos does not)
  return atan2f(vec3Norm(vec3Cross(a, b)), vec3Dot(a, b));
}

#endif // VEC_MATH_H
