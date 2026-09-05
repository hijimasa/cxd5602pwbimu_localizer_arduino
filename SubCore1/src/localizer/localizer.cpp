#include "localizer.h"

static const float RAD_TO_DEG_F = 180.0f / LOCALIZER_PI;
static const float DEG_TO_RAD_F = LOCALIZER_PI / 180.0f;

Localizer::Localizer()
    : alignmentAccumulator_(LOCALIZER_CALIBRATION_SAMPLES)
{
  earthRateCompensation_ = true;
  biasLearning_ = true;
  gainScheduling_ = true;
  accelBiasPreset_ = vec3Zero(); // survives reset(): it is a device constant
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

  FusionOffsetInitialise(&offset_, LOCALIZER_SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs_);
  const FusionAhrsSettings settings = {
      .convention = FusionConventionNwu,
      .gain = FUSION_AHRS_GAIN,
      .gyroscopeRange = FUSION_GYRO_RANGE,
      .accelerationRejection = FUSION_ACCEL_REJECTION,
      .magneticRejection = 0.0f,
      .recoveryTriggerPeriod = FUSION_RECOVERY_TRIGGER_PERIOD,
  };
  FusionAhrsSetSettings(&ahrs_, &settings);
  currentGain_ = FUSION_AHRS_GAIN;
  ambiguousCounter_ = 0;

  zupt_.reset();
  integrator_.reset();

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
  return vec3(offset_.gyroscopeOffset.axis.x * DEG_TO_RAD_F,
              offset_.gyroscopeOffset.axis.y * DEG_TO_RAD_F,
              offset_.gyroscopeOffset.axis.z * DEG_TO_RAD_F);
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
  alignment_ = alignmentAccumulator_.compute(LOCALIZER_GRAVITY, LOCALIZER_LATITUDE_DEG, LOCALIZER_EARTH_RATE);

  // Seed the AHRS with the aligned attitude and skip Fusion's own
  // initialisation ramp (the attitude is already known).
  FusionQuaternion q;
  q.element.w = alignment_.quaternion.w;
  q.element.x = alignment_.quaternion.x;
  q.element.y = alignment_.quaternion.y;
  q.element.z = alignment_.quaternion.z;
  FusionAhrsSetQuaternion(&ahrs_, q);
  ahrs_.initialising = false;
  ahrs_.rampedGain = FUSION_AHRS_GAIN;

  // Seed the run-time gyro offset tracker (deg/s) with the measured offset.
  offset_.gyroscopeOffset.axis.x = alignment_.gyroOffset.x * RAD_TO_DEG_F;
  offset_.gyroscopeOffset.axis.y = alignment_.gyroOffset.y * RAD_TO_DEG_F;
  offset_.gyroscopeOffset.axis.z = alignment_.gyroOffset.z * RAD_TO_DEG_F;

  accelBias_ = alignment_.accelBias;

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
      .convention = FusionConventionNwu,
      .gain = gain,
      .gyroscopeRange = FUSION_GYRO_RANGE,
      .accelerationRejection = FUSION_ACCEL_REJECTION,
      .magneticRejection = 0.0f,
      .recoveryTriggerPeriod = FUSION_RECOVERY_TRIGGER_PERIOD,
  };
  FusionAhrsSetSettings(&ahrs_, &settings); // initialising is false: rampedGain = gain
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

  // 3. Run-time gyro offset tracking (Fusion works in deg/s)
  FusionVector gyroscope = {{gyroComp.x * RAD_TO_DEG_F, gyroComp.y * RAD_TO_DEG_F, gyroComp.z * RAD_TO_DEG_F}};
  gyroscope = FusionOffsetUpdate(&offset_, gyroscope);

  // 4. AHRS update (accelerometer in g, without bias correction so the
  //    attitude tracks the measured gravity direction)
  const FusionVector accelerometer = {{accelF.x / LOCALIZER_GRAVITY, accelF.y / LOCALIZER_GRAVITY, accelF.z / LOCALIZER_GRAVITY}};
  FusionAhrsUpdateNoMagnetometer(&ahrs_, gyroscope, accelerometer, dt);

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
