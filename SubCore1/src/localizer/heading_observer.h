/**
 * @file heading_observer.h
 * @brief Gyro-compass re-estimation of the heading and the gyro bias from
 *        the stationary periods of a run (no Arduino dependency).
 *
 * At rest the gyroscope measures  m = b + R^T w_e  (b: bias, R: attitude,
 * w_e: earth rotation). The initial alignment takes North from one such
 * mean after removing the calibrated bias preset. A preset that is off by
 * more than the horizontal earth rate (0.0034 deg/s in Tokyo) gives an
 * arbitrary heading, and nothing corrected it afterwards: the run-time
 * offset tracker absorbed the residual of a wrong heading as "bias".
 *
 * This observer remembers the gyro mean of the recent stationary periods
 * together with the AHRS attitude R_i at the end of each one and fits, by
 * weighted least squares,
 *     m_i = b + R_i^T Rz(-psi) w_e
 * for the bias b (3 unknowns) and the yaw error psi of the AHRS frame
 * (as cos psi, sin psi: 2 unknowns, linear). The bias preset is a prior
 * with its own uncertainty, every period is weighted by its length (the
 * noise of a mean falls as 1/sqrt(T)) and loses weight with age
 * (temperature drift). With periods at one heading only, the prior decides
 * the split between bias and heading, which reproduces the alignment. Once
 * periods at different headings exist the bias becomes observable (it is
 * fixed to the sensor while the earth rate turns with the heading) and the
 * heading follows the data; e.g. resting once with the x axis East and once
 * with it West separates a y-axis bias from the earth rate completely.
 *
 * The localizer applies the yaw correction as a slow rotation of its frame
 * about Up (attitude and integrated position alike) and reports it back
 * with applyYaw() so that the stored attitudes stay in the current frame.
 */
#ifndef HEADING_OBSERVER_H
#define HEADING_OBSERVER_H

#include <stdint.h>
#include "localizer_config.h"
#include "vec_math.h"

struct HeadingObserverStatus
{
  int periods;             // stored stationary periods (the alignment counts as one)
  float currentPeriodTime; // s, length of the stationary period in progress
  float yawPending;        // rad, correction asked for by the last solve, not applied yet
  float yawApplied;        // rad, total correction applied since reset
  Vec3 bias;               // rad/s, current gyro bias estimate (sensor frame)
  float residualRms;       // rad/s, fit residual over the stored periods
  float yawScale;          // horizontal rate seen by the fit in units of the horizontal earth rate (1 when the model fits)
  float rateMagnitude;     // rad/s, |mean - bias| of the latest period (earth rate when consistent)
  float verticalRate;      // rad/s, Up component of (mean - bias) of the latest period
  uint32_t solves;         // fits performed
  uint32_t rejectedPeriods; // periods dropped because the mean was not a rest
};

class HeadingObserver
{
public:
  HeadingObserver();

  /**
   * Clear everything and set the model.
   * @param biasPrior   gyro bias preset (rad/s, sensor frame)
   * @param latitudeDeg latitude of the site
   * @param earthRate   earth rotation rate (rad/s)
   */
  void reset(Vec3 biasPrior, float latitudeDeg, float earthRate);

  /** Advance the clock by one sample (ages the stored periods). */
  void advance(float dt);

  /** One stationary sample (gyro in rad/s, bias not removed). */
  void addStationary(Vec3 gyro, float dt);

  /**
   * The sensor started moving: close the period in progress. The attitude
   * is the AHRS attitude at the end of the rest. Short periods are dropped.
   */
  void endPeriod(Quat attitude);

  /** Add a complete period directly (used for the initial alignment). */
  void addPeriod(Vec3 meanGyro, float duration, Quat attitude);

  /**
   * Fit bias and yaw error from the stored periods, the period in progress
   * (if long enough; `attitude` is the current AHRS attitude) and the prior.
   * @return false if there is nothing to fit
   */
  bool solve(Quat attitude);

  /** Yaw correction (rad, about Up, applied as q <- Rz(yaw) q) still to apply. */
  float pendingYaw() const { return status_.yawPending; }

  /** The frame was rotated about Up by delta (rad): keep the stored attitudes current. */
  void applyYaw(float delta);

  Vec3 bias() const { return status_.bias; }
  /** Length of the rest in progress that is used (head and tail excluded). */
  float currentPeriodTime() const { return status_.currentPeriodTime; }
  const HeadingObserverStatus &status() const { return status_; }

  /** Weight of a period of the given length and age (relative to a 3 s mean). */
  static float periodWeight(float duration, float age);

  struct Chunk
  {
    double sum[3];
    uint32_t samples;
  };

  struct Period
  {
    Vec3 mean;      // rad/s
    float duration; // s
    float age;      // s since the period ended
    Quat attitude;  // sensor relative to Earth, in the current (corrected) frame
  };
  /** Stored periods, oldest first (diagnostics). */
  int periodCount() const { return count_; }
  const Period &period(int i) const { return periods_[(next_ - count_ + i + HEADING_OBSERVER_PERIODS) % HEADING_OBSERVER_PERIODS]; }
  /** Expected gyro mean at rest for an attitude with the current bias. */
  Vec3 expectedMean(Quat attitude) const;

private:
  bool storePeriod(Vec3 mean, float duration, Quat attitude);
  bool looksLikeRest(Vec3 mean) const;
  Vec3 currentMean() const;

  Period periods_[HEADING_OBSERVER_PERIODS];
  int count_;
  int next_;

  // The period in progress: committed samples plus a short tail held back
  // in chunks, so that the start of the next motion (which the ZUPT
  // detector only notices after its window) can be discarded at the end.
  void commitChunk();
  void clearCurrent();
  double sum_[3];
  uint32_t samples_;
  Chunk tail_[HEADING_OBSERVER_TAIL_CHUNKS];
  Chunk chunk_;
  uint32_t headSkipped_;
  float time_;

  Vec3 prior_;
  float horizontalRate_;
  float verticalRate_;
  HeadingObserverStatus status_;
};

/** Rotation about the Earth Up axis by angle (rad), as a quaternion. */
static inline Quat quatAboutUp(float angle)
{
  const float h = 0.5f * angle;
  Quat q = {cosf(h), 0.0f, 0.0f, sinf(h)};
  return q;
}

#endif // HEADING_OBSERVER_H
