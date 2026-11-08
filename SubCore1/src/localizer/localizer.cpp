#include "localizer.h"

static const float RAD_TO_DEG_F = 180.0f / LOCALIZER_PI;
static const float DEG_TO_RAD_F = LOCALIZER_PI / 180.0f;

Localizer::Localizer()
    : alignmentAccumulator_(LOCALIZER_CALIBRATION_SAMPLES)
{
  earthRateCompensation_ = true;
  biasLearning_ = true;
  gainScheduling_ = true;
  headingObserverEnabled_ = HEADING_OBSERVER_ENABLED != 0;
  accelBiasPreset_ = vec3Zero(); // survives reset(): it is a device constant
  gyroBiasPreset_ = vec3Zero();
  hasGyroBiasPreset_ = false;
  reset();
}

void Localizer::reset()
{
  phase_ = CALIBRATING;
  alignmentAccumulator_.reset();

  for (int i = 0; i < 3; i++)
  {
    accelFilter_[i].reset();
    gyroFilter_[i].reset();
  }

  FusionBiasInitialise(&bias_);
  const FusionBiasSettings biasSettings = {
      .sampleRate = (float)LOCALIZER_SAMPLE_RATE,
      .stationaryThreshold = FUSION_BIAS_STATIONARY_THRESHOLD,
      .stationaryPeriod = FUSION_BIAS_STATIONARY_PERIOD,
  };
  FusionBiasSetSettings(&bias_, &biasSettings);
  FusionAhrsInitialise(&ahrs_);
  const FusionAhrsSettings settings = {
      .sampleRate = (float)LOCALIZER_SAMPLE_RATE,
      .convention = FusionConventionNwu,
      .gain = FUSION_AHRS_GAIN,
      .gyroscopeRange = FUSION_GYRO_RANGE,
      .accelerationRejection = FUSION_ACCEL_REJECTION,
      .magneticRejection = 0.0f,
      .rejectionTimeout = FUSION_REJECTION_TIMEOUT,
  };
  FusionAhrsSetSettings(&ahrs_, &settings);
  currentGain_ = FUSION_AHRS_GAIN;
  ambiguousCounter_ = 0;

  zupt_.reset();
  integrator_.reset();
  headingObserver_.reset(gyroBiasPreset_, LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE);
  observerSolveCounter_ = 0;

  accelBias_ = vec3Zero();
  earthRateNwu_ = earthRateNwu(LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE);

  lastTimestamp_ = 0;
  haveLastTimestamp_ = false;

  state_.timestamp = 0;
  state_.quaternion = quatIdentity();
  state_.velocity = vec3Zero();
  state_.position = vec3Zero();
  state_.earthAccel = vec3Zero();
  state_.sensorAccel = vec3Zero();
  state_.gyro = vec3Zero();
  state_.dt = LOCALIZER_NOMINAL_DT;
  state_.temp = 0.0f;
  state_.stationary = false;

  diag_.samplesProcessed = 0;
  diag_.dtAnomalies = 0;
  diag_.lastDt = LOCALIZER_NOMINAL_DT;
  diag_.maxDt = 0.0f;
}

Vec3 Localizer::gyroOffsetEstimate() const
{
  if (headingObserverActive()) return headingObserver_.bias();
  const FusionVector offset = FusionBiasGetOffset(&bias_);
  return vec3(offset.axis.x * DEG_TO_RAD_F, offset.axis.y * DEG_TO_RAD_F, offset.axis.z * DEG_TO_RAD_F);
}

bool Localizer::update(const ImuSample &rawSample)
{
  ImuSample sample = rawSample;
  sample.accel = vec3Sub(sample.accel, accelBiasPreset_);
  if (phase_ == CALIBRATING)
  {
    if (alignmentAccumulator_.add(sample.accel, sample.gyro))
    {
      startRunning(sample);
    }
    return false;
  }
  processRunning(sample);
  return true;
}

void Localizer::startRunning(const ImuSample &sample)
{
  alignment_ = alignmentAccumulator_.compute(LOCALIZER_GRAVITY, LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE,
                                             hasGyroBiasPreset_ ? &gyroBiasPreset_ : 0);

  // Seed the AHRS with the aligned attitude and skip Fusion's own
  // start-up gain ramp (the attitude is already known).
  FusionQuaternion q;
  q.element.w = alignment_.quaternion.w;
  q.element.x = alignment_.quaternion.x;
  q.element.y = alignment_.quaternion.y;
  q.element.z = alignment_.quaternion.z;
  FusionAhrsSetQuaternion(&ahrs_, q);
  FusionAhrsSkipStartup(&ahrs_);

  // Seed the run-time gyro offset tracker (deg/s) with the measured offset.
  const FusionVector offset = {{alignment_.gyroOffset.x * RAD_TO_DEG_F,
                                alignment_.gyroOffset.y * RAD_TO_DEG_F,
                                alignment_.gyroOffset.z * RAD_TO_DEG_F}};
  FusionBiasSetOffset(&bias_, offset);

  accelBias_ = alignment_.accelBias;

  // The alignment is the first stationary period of the heading observer.
  headingObserver_.reset(gyroBiasPreset_, LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE);
  if (headingObserverActive())
  {
    headingObserver_.addPeriod(alignment_.meanGyro,
                               (float)alignmentAccumulator_.requiredSamples() / LOCALIZER_SAMPLE_RATE,
                               alignment_.quaternion);
    headingObserver_.solve(alignment_.quaternion);
  }

  // The first running sample uses the nominal dt.
  lastTimestamp_ = sample.timestamp;
  haveLastTimestamp_ = false;
  phase_ = RUNNING;
}

void Localizer::applyAhrsGain(float gain)
{
  if (gain == currentGain_)
  {
    return;
  }
  const FusionAhrsSettings settings = {
      .sampleRate = (float)LOCALIZER_SAMPLE_RATE,
      .convention = FusionConventionNwu,
      .gain = gain,
      .gyroscopeRange = FUSION_GYRO_RANGE,
      .accelerationRejection = FUSION_ACCEL_REJECTION,
      .magneticRejection = 0.0f,
      .rejectionTimeout = FUSION_REJECTION_TIMEOUT,
  };
  FusionAhrsSetSettings(&ahrs_, &settings); // start-up is over: the gain applies at once
  currentGain_ = gain;
}

float Localizer::computeDt(uint32_t timestamp)
{
  float dt = LOCALIZER_NOMINAL_DT;
  if (haveLastTimestamp_)
  {
    // unsigned subtraction handles the 32-bit wrap-around
    const uint32_t ticks = timestamp - lastTimestamp_;
    dt = (float)ticks / LOCALIZER_TIMESTAMP_HZ;
    if (dt > LOCALIZER_DT_ANOMALY_FACTOR * LOCALIZER_NOMINAL_DT)
    {
      diag_.dtAnomalies++;
    }
    if (dt > LOCALIZER_MAX_DT)
    {
      dt = LOCALIZER_MAX_DT;
    }
    if (dt <= 0.0f)
    {
      dt = LOCALIZER_NOMINAL_DT;
    }
  }
  lastTimestamp_ = timestamp;
  haveLastTimestamp_ = true;
  diag_.lastDt = dt;
  if (dt > diag_.maxDt)
  {
    diag_.maxDt = dt;
  }
  return dt;
}

void Localizer::processRunning(const ImuSample &sample)
{
  const float dt = computeDt(sample.timestamp);

  // 1. Causal Gaussian pre-filter
  const Vec3 accelF = vec3(accelFilter_[0].update(sample.accel.x),
                           accelFilter_[1].update(sample.accel.y),
                           accelFilter_[2].update(sample.accel.z));
  const Vec3 gyroF = vec3(gyroFilter_[0].update(sample.gyro.x),
                          gyroFilter_[1].update(sample.gyro.y),
                          gyroFilter_[2].update(sample.gyro.z));

  // 2. Earth rotation compensation using the current attitude estimate
  Vec3 gyroComp = gyroF;
  if (earthRateCompensation_)
  {
    gyroComp = vec3Sub(gyroComp, quatRotateEarthToSensor(state_.quaternion, earthRateNwu_));
  }

  // 3. Run-time gyro offset (Fusion works in deg/s). With the heading
  //    observer the offset is its bias estimate; FusionBias would otherwise
  //    absorb the residual of a wrong heading at rest and freeze the error.
  const bool observer = headingObserverActive();
  FusionVector gyroscope;
  if (observer)
  {
    const Vec3 g = vec3Sub(gyroComp, headingObserver_.bias());
    gyroscope.axis.x = g.x * RAD_TO_DEG_F;
    gyroscope.axis.y = g.y * RAD_TO_DEG_F;
    gyroscope.axis.z = g.z * RAD_TO_DEG_F;
  }
  else
  {
    const FusionVector raw = {{gyroComp.x * RAD_TO_DEG_F, gyroComp.y * RAD_TO_DEG_F, gyroComp.z * RAD_TO_DEG_F}};
    gyroscope = FusionBiasUpdate(&bias_, raw);
  }

  // 4. AHRS update (accelerometer in g, without bias correction so the
  //    attitude tracks the measured gravity direction)
  const FusionVector accelerometer = {{accelF.x / LOCALIZER_GRAVITY, accelF.y / LOCALIZER_GRAVITY, accelF.z / LOCALIZER_GRAVITY}};
  FusionAhrsSetSamplePeriod(&ahrs_, dt); // measured dt (SetSettings resets it to the nominal value)
  FusionAhrsUpdateNoMagnetometer(&ahrs_, gyroscope, accelerometer);

  const FusionQuaternion fq = FusionAhrsGetQuaternion(&ahrs_);
  Quat q = {fq.element.w, fq.element.x, fq.element.y, fq.element.z};

  // 5. Gravity removal and bias correction in the sensor frame, then rotate
  const FusionVector linear = FusionAhrsGetLinearAcceleration(&ahrs_);
  const Vec3 linearRaw = vec3(linear.axis.x * LOCALIZER_GRAVITY, linear.axis.y * LOCALIZER_GRAVITY, linear.axis.z * LOCALIZER_GRAVITY);
  const Vec3 sensorAccel = vec3Sub(linearRaw, accelBias_);
  const Vec3 earthAccel = quatRotateSensorToEarth(q, sensorAccel);

  // 6. ZUPT detection on the corrected signals
  const Vec3 gyroCorrected = vec3(gyroscope.axis.x * DEG_TO_RAD_F, gyroscope.axis.y * DEG_TO_RAD_F, gyroscope.axis.z * DEG_TO_RAD_F);
  const bool stationary = zupt_.update(earthAccel, gyroCorrected, vec3Norm(integrator_.velocity()));

  // 6b. AHRS gain scheduling for the next update
  if (gainScheduling_)
  {
    if (stationary)
    {
      ambiguousCounter_ = 0;
      applyAhrsGain(FUSION_GAIN_STATIONARY);
    }
    else if (!zupt_.isLowVariance())
    {
      ambiguousCounter_ = 0;
      applyAhrsGain(FUSION_GAIN_MOTION);
    }
    else if (zupt_.isLowMean())
    {
      ambiguousCounter_ = 0;
      applyAhrsGain(FUSION_GAIN_SETTLING);
    }
    else
    {
      // smooth sustained acceleration, or an attitude error at rest
      if (ambiguousCounter_ < FUSION_AMBIGUOUS_TIMEOUT)
      {
        ambiguousCounter_++;
      }
      applyAhrsGain(ambiguousCounter_ >= FUSION_AMBIGUOUS_TIMEOUT ? FUSION_GAIN_SETTLING : FUSION_GAIN_MOTION);
    }
  }

  // 7. Accel bias learning while stationary (residual should be zero)
  if (stationary && biasLearning_)
  {
    accelBias_ = vec3Add(accelBias_, vec3Scale(sensorAccel, BIAS_LEARNING_RATE));
  }

  // 8. Velocity / position integration
  integrator_.update(earthAccel, dt, stationary, zupt_.stillSamples() * LOCALIZER_NOMINAL_DT);

  // 9. Heading observer: fit bias and heading from the rest periods and
  //    turn the attitude slowly about Up. The integrated position is left
  //    alone on purpose: a robot acting on it must not see it move while
  //    the sensor rests, even though the path integrated before a large
  //    correction then stays in the old frame.
  if (observer)
  {
    headingObserver_.advance(dt);
    if (stationary)
    {
      headingObserver_.addStationary(gyroF, dt);
      // Solve every interval once the rest is long enough to count; a
      // brief ZUPT during a slow move must not move the heading.
      const bool usable = headingObserver_.currentPeriodTime() >= HEADING_OBSERVER_MIN_PERIOD;
      if (usable && ++observerSolveCounter_ >= HEADING_OBSERVER_SOLVE_INTERVAL)
      {
        observerSolveCounter_ = 0;
        headingObserver_.solve(q);
      }
      float delta = usable ? headingObserver_.pendingYaw() * dt / HEADING_OBSERVER_CORRECTION_TIME : 0.0f;
      const float limit = HEADING_OBSERVER_MAX_RATE * dt;
      if (delta > limit) delta = limit;
      if (delta < -limit) delta = -limit;
      if (delta != 0.0f)
      {
        q = quatNormalise(quatMultiply(quatAboutUp(delta), q));
        const FusionQuaternion fq2 = {{q.w, q.x, q.y, q.z}};
        FusionAhrsSetQuaternion(&ahrs_, fq2);
        headingObserver_.applyYaw(delta);
      }
    }
    else
    {
      observerSolveCounter_ = 0;
      headingObserver_.endPeriod(q);
    }
  }

  state_.timestamp = sample.timestamp;
  state_.quaternion = q;
  state_.velocity = integrator_.velocity();
  state_.position = integrator_.position();
  state_.earthAccel = earthAccel;
  state_.sensorAccel = sensorAccel;
  state_.gyro = gyroCorrected;
  state_.dt = dt;
  state_.temp = sample.temp;
  state_.stationary = stationary;

  diag_.samplesProcessed++;
}
