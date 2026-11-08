#include "heading_observer.h"

/**
 * Gauss-Jordan solve of a small dense system a x = y (row-major n x n).
 * The pivot threshold is relative to the largest diagonal entry.
 */
static bool solveSmall(int n, double *a, double *y, double *x, double minPivotRel)
{
  double largest = 0.0;
  for (int i = 0; i < n; i++)
    if (a[i * n + i] > largest) largest = a[i * n + i];
  const double minPivot = minPivotRel * largest;
  for (int c = 0; c < n; c++)
  {
    int piv = c;
    for (int r = c + 1; r < n; r++)
      if (fabs(a[r * n + c]) > fabs(a[piv * n + c])) piv = r;
    if (fabs(a[piv * n + c]) <= minPivot) return false;
    if (piv != c)
    {
      for (int j = 0; j < n; j++)
      {
        const double t = a[c * n + j];
        a[c * n + j] = a[piv * n + j];
        a[piv * n + j] = t;
      }
      const double t = y[c];
      y[c] = y[piv];
      y[piv] = t;
    }
    for (int r = 0; r < n; r++)
    {
      if (r == c) continue;
      const double f = a[r * n + c] / a[c * n + c];
      if (f == 0.0) continue;
      for (int j = c; j < n; j++) a[r * n + j] -= f * a[c * n + j];
      y[r] -= f * y[c];
    }
  }
  for (int i = 0; i < n; i++) x[i] = y[i] / a[i * n + i];
  return true;
}

HeadingObserver::HeadingObserver()
{
  reset(vec3Zero(), LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE);
}

static void clearChunk(HeadingObserver::Chunk &c)
{
  c.sum[0] = c.sum[1] = c.sum[2] = 0.0;
  c.samples = 0;
}

void HeadingObserver::clearCurrent()
{
  sum_[0] = sum_[1] = sum_[2] = 0.0;
  samples_ = 0;
  for (int i = 0; i < HEADING_OBSERVER_TAIL_CHUNKS; i++) clearChunk(tail_[i]);
  clearChunk(chunk_);
  headSkipped_ = 0;
  time_ = 0.0f;
  status_.currentPeriodTime = 0.0f;
}

void HeadingObserver::reset(Vec3 biasPrior, float latitudeDeg, float earthRate)
{
  count_ = 0;
  clearCurrent();
  prior_ = biasPrior;
  const float lat = latitudeDeg * LOCALIZER_PI / 180.0f;
  horizontalRate_ = earthRate * cosf(lat);
  verticalRate_ = earthRate * sinf(lat);

  status_.periods = 0;
  status_.currentPeriodTime = 0.0f;
  status_.yawPending = 0.0f;
  status_.yawApplied = 0.0f;
  status_.bias = biasPrior;
  status_.residualRms = 0.0f;
  status_.yawScale = 1.0f;
  status_.rateMagnitude = 0.0f;
  status_.verticalRate = 0.0f;
  status_.solves = 0;
  status_.rejectedPeriods = 0;
}

float HeadingObserver::periodWeight(float duration, float age)
{
  float t = duration;
  if (t > HEADING_OBSERVER_MAX_PERIOD) t = HEADING_OBSERVER_MAX_PERIOD;
  return (t / HEADING_OBSERVER_NOISE_PERIOD) * expf(-age / HEADING_OBSERVER_MEMORY);
}

void HeadingObserver::advance(float dt)
{
  for (int i = 0; i < count_; i++) periods_[i].age += dt;
}

void HeadingObserver::commitChunk()
{
  // the oldest held-back chunk becomes committed, the current chunk joins the tail
  Chunk &oldest = tail_[0];
  for (int k = 0; k < 3; k++) sum_[k] += oldest.sum[k];
  samples_ += oldest.samples;
  for (int i = 0; i + 1 < HEADING_OBSERVER_TAIL_CHUNKS; i++) tail_[i] = tail_[i + 1];
  tail_[HEADING_OBSERVER_TAIL_CHUNKS - 1] = chunk_;
  clearChunk(chunk_);
}

void HeadingObserver::addStationary(Vec3 gyro, float dt)
{
  if (headSkipped_ < HEADING_OBSERVER_HEAD_SAMPLES)
  {
    headSkipped_++;
    return;
  }
  chunk_.sum[0] += gyro.x;
  chunk_.sum[1] += gyro.y;
  chunk_.sum[2] += gyro.z;
  chunk_.samples++;
  if (chunk_.samples >= HEADING_OBSERVER_TAIL_CHUNK) commitChunk();
  time_ += dt;
  status_.currentPeriodTime = time_;
}

/** Mean of everything seen in the period so far (tail included). */
Vec3 HeadingObserver::currentMean() const
{
  double sum[3] = {sum_[0], sum_[1], sum_[2]};
  uint32_t n = samples_;
  for (int i = 0; i < HEADING_OBSERVER_TAIL_CHUNKS; i++)
  {
    for (int k = 0; k < 3; k++) sum[k] += tail_[i].sum[k];
    n += tail_[i].samples;
  }
  for (int k = 0; k < 3; k++) sum[k] += chunk_.sum[k];
  n += chunk_.samples;
  if (n == 0) return vec3Zero();
  return vec3((float)(sum[0] / n), (float)(sum[1] / n), (float)(sum[2] / n));
}

void HeadingObserver::endPeriod(Quat attitude)
{
  // Without the tail: the ZUPT detector reports the start of a motion
  // only after its window, so the last fraction of a second may already
  // contain the first rotation.
  if (samples_ > 0)
  {
    uint32_t total = samples_ + chunk_.samples;
    for (int i = 0; i < HEADING_OBSERVER_TAIL_CHUNKS; i++) total += tail_[i].samples;
    const float duration = time_ * ((float)samples_ / (float)total);
    if (duration >= HEADING_OBSERVER_MIN_PERIOD)
    {
      const Vec3 mean = vec3((float)(sum_[0] / samples_), (float)(sum_[1] / samples_), (float)(sum_[2] / samples_));
      storePeriod(mean, duration, attitude);
    }
  }
  clearCurrent();
  // A correction is only applied on the strength of a fit that includes
  // the rest in progress: whatever was left is dropped with the motion.
  status_.yawPending = 0.0f;
}

void HeadingObserver::addPeriod(Vec3 meanGyro, float duration, Quat attitude)
{
  storePeriod(meanGyro, duration, attitude);
}

bool HeadingObserver::looksLikeRest(Vec3 mean) const
{
  return vec3Norm(vec3Sub(mean, status_.bias)) < HEADING_OBSERVER_MAX_RESIDUAL;
}

bool HeadingObserver::storePeriod(Vec3 mean, float duration, Quat attitude)
{
  if (!looksLikeRest(mean))
  {
    status_.rejectedPeriods++;
    return false;
  }
  int slot = count_;
  if (count_ >= HEADING_OBSERVER_PERIODS)
  {
    // full: replace the period that contributes least (short or old), so
    // that a long rest such as the alignment is not pushed out by a run of
    // brief stops
    slot = 0;
    float lightest = periodWeight(periods_[0].duration, periods_[0].age);
    for (int i = 1; i < count_; i++)
    {
      const float w = periodWeight(periods_[i].duration, periods_[i].age);
      if (w < lightest)
      {
        lightest = w;
        slot = i;
      }
    }
  }
  else
  {
    count_++;
  }
  Period &p = periods_[slot];
  p.mean = mean;
  p.duration = duration;
  p.age = 0.0f;
  p.attitude = attitude;
  status_.periods = count_;
  const Vec3 r = vec3Sub(mean, status_.bias);
  status_.rateMagnitude = vec3Norm(r);
  status_.verticalRate = vec3Dot(r, quatRotateEarthToSensor(attitude, vec3(0.0f, 0.0f, 1.0f)));
  return true;
}

bool HeadingObserver::solve(Quat attitude)
{
  // Unknowns, scaled by the horizontal earth rate: x = (b/Wh, cos psi, sin psi)
  //   m_i/Wh - (Wv/Wh) z_i = b/Wh + cos psi * n_i - sin psi * w_i
  // with n_i, w_i, z_i the Earth North/West/Up axes in the sensor frame.
  // The linear solve gives the start; cos/sin are then held on the unit
  // circle by alternating closed-form updates of the bias and the yaw
  // (otherwise two rests at different headings can be fitted by shrinking
  // the horizontal earth rate instead of moving the bias).
  struct Row
  {
    Vec3 mean, n, w, z;
    float weight;
  };
  Row rowsBuf[HEADING_OBSERVER_PERIODS + 1];
  int rows = 0;
  for (int i = 0; i <= count_; i++)
  {
    Row &r = rowsBuf[rows];
    Quat q;
    if (i < count_)
    {
      r.mean = periods_[i].mean;
      r.weight = periodWeight(periods_[i].duration, periods_[i].age);
      q = periods_[i].attitude;
    }
    else
    {
      if (time_ < HEADING_OBSERVER_MIN_PERIOD) break;
      r.mean = currentMean();
      if (!looksLikeRest(r.mean)) break;
      r.weight = periodWeight(time_, 0.0f);
      q = attitude;
    }
    r.n = quatRotateEarthToSensor(q, vec3(1.0f, 0.0f, 0.0f));
    r.w = quatRotateEarthToSensor(q, vec3(0.0f, 1.0f, 0.0f));
    r.z = quatRotateEarthToSensor(q, vec3(0.0f, 0.0f, 1.0f));
    rows++;
  }
  if (rows == 0) return false;

  const double wh = horizontalRate_;
  const double wv = verticalRate_;
  const double wp = (double)(HEADING_OBSERVER_MEAN_NOISE / HEADING_OBSERVER_PRESET_SIGMA) *
                    (double)(HEADING_OBSERVER_MEAN_NOISE / HEADING_OBSERVER_PRESET_SIGMA);
  const double priorScaled[3] = {prior_.x / wh, prior_.y / wh, prior_.z / wh};

  // 1. Linear least squares in (b/Wh, c, s)
  double ata[25] = {0.0};
  double aty[5] = {0.0};
  for (int k = 0; k < 3; k++)
  {
    ata[k * 5 + k] += wp;
    aty[k] += wp * priorScaled[k];
  }
  for (int i = 0; i < rows; i++)
  {
    const Row &r = rowsBuf[i];
    const float nk[3] = {r.n.x, r.n.y, r.n.z}, wk[3] = {r.w.x, r.w.y, r.w.z}, zk[3] = {r.z.x, r.z.y, r.z.z};
    const float mk[3] = {r.mean.x, r.mean.y, r.mean.z};
    for (int k = 0; k < 3; k++)
    {
      double row[5] = {0.0, 0.0, 0.0, nk[k], -wk[k]};
      row[k] = 1.0;
      const double y = mk[k] / wh - (wv / wh) * zk[k];
      for (int p = 0; p < 5; p++)
      {
        aty[p] += r.weight * row[p] * y;
        for (int c = 0; c < 5; c++) ata[p * 5 + c] += r.weight * row[p] * row[c];
      }
    }
  }
  double x[5];
  if (!solveSmall(5, ata, aty, x, 1.0e-9)) return false;
  double c = x[3], s = x[4];
  const double scale0 = sqrt(c * c + s * s);
  if (scale0 < 1.0e-6) return false;
  c /= scale0;
  s /= scale0;
  double b[3] = {x[0], x[1], x[2]};

  // 2. Alternate: bias given the yaw (weighted mean with the prior), yaw
  //    given the bias (direction of the weighted horizontal residual)
  double scale = scale0;
  for (int it = 0; it < 4; it++)
  {
    double num[3] = {wp * priorScaled[0], wp * priorScaled[1], wp * priorScaled[2]};
    double den = wp;
    for (int i = 0; i < rows; i++)
    {
      const Row &r = rowsBuf[i];
      const float nk[3] = {r.n.x, r.n.y, r.n.z}, wk[3] = {r.w.x, r.w.y, r.w.z}, zk[3] = {r.z.x, r.z.y, r.z.z};
      const float mk[3] = {r.mean.x, r.mean.y, r.mean.z};
      for (int k = 0; k < 3; k++)
        num[k] += r.weight * (mk[k] / wh - (wv / wh) * zk[k] - c * nk[k] + s * wk[k]);
      den += r.weight;
    }
    for (int k = 0; k < 3; k++) b[k] = num[k] / den;

    double sumN = 0.0, sumW = 0.0, sumWeight = 0.0;
    for (int i = 0; i < rows; i++)
    {
      const Row &r = rowsBuf[i];
      const double res[3] = {r.mean.x / wh - (wv / wh) * r.z.x - b[0],
                             r.mean.y / wh - (wv / wh) * r.z.y - b[1],
                             r.mean.z / wh - (wv / wh) * r.z.z - b[2]};
      sumN += r.weight * (res[0] * r.n.x + res[1] * r.n.y + res[2] * r.n.z);
      sumW -= r.weight * (res[0] * r.w.x + res[1] * r.w.y + res[2] * r.w.z);
      sumWeight += r.weight;
    }
    const double mag = sqrt(sumN * sumN + sumW * sumW);
    if (mag < 1.0e-6) break;
    c = sumN / mag;
    s = sumW / mag;
    scale = mag / sumWeight; // horizontal rate seen, in units of the earth rate
  }

  status_.solves++;
  status_.bias = vec3((float)(b[0] * wh), (float)(b[1] * wh), (float)(b[2] * wh));
  status_.yawScale = (float)scale;
  status_.yawPending = atan2f((float)s, (float)c);

  // Residual over the stored periods with the fitted model
  double sumSq = 0.0;
  for (int i = 0; i < count_; i++)
  {
    const Row &r = rowsBuf[i];
    const Vec3 expected = vec3Add(status_.bias,
                                  vec3Add(vec3Scale(vec3Sub(vec3Scale(r.n, (float)c), vec3Scale(r.w, (float)s)), horizontalRate_),
                                          vec3Scale(r.z, verticalRate_)));
    const Vec3 d = vec3Sub(r.mean, expected);
    sumSq += vec3Dot(d, d);
  }
  status_.residualRms = count_ > 0 ? sqrtf((float)(sumSq / count_)) : 0.0f;
  return true;
}

Vec3 HeadingObserver::expectedMean(Quat attitude) const
{
  const Vec3 n = quatRotateEarthToSensor(attitude, vec3(1.0f, 0.0f, 0.0f));
  const Vec3 z = quatRotateEarthToSensor(attitude, vec3(0.0f, 0.0f, 1.0f));
  return vec3Add(status_.bias, vec3Add(vec3Scale(n, horizontalRate_), vec3Scale(z, verticalRate_)));
}

void HeadingObserver::applyYaw(float delta)
{
  if (delta == 0.0f) return;
  const Quat rz = quatAboutUp(delta);
  for (int i = 0; i < count_; i++)
  {
    periods_[i].attitude = quatNormalise(quatMultiply(rz, periods_[i].attitude));
  }
  status_.yawPending -= delta;
  status_.yawApplied += delta;
}
