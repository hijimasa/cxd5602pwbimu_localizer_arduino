#include "zupt_detector.h"

ZuptDetector::ZuptDetector()
{
  reset();
}

void ZuptDetector::reset()
{
  for (int a = 0; a < 3; a++)
  {
    for (int i = 0; i < ZUPT_WINDOW_SIZE; i++)
    {
      accelBuffer_[a][i] = 0.0f;
      gyroBuffer_[a][i] = 0.0f;
    }
  }
  index_ = 0;
  filled_ = false;
  stillCounter_ = 0;
  stationary_ = false;
  lowVariance_ = false;
  lowMean_ = false;
  accelVariance_ = 0.0f;
  gyroVariance_ = 0.0f;
  accelMeanNorm_ = 0.0f;
  gyroMeanNorm_ = 0.0f;
}

void ZuptDetector::meanAndVariance(const float *buffer, int size, float &mean, float &variance)
{
  mean = 0.0f;
  for (int i = 0; i < size; i++)
  {
    mean += buffer[i];
  }
  mean /= size;
  variance = 0.0f;
  for (int i = 0; i < size; i++)
  {
    const float d = buffer[i] - mean;
    variance += d * d;
  }
  variance /= size;
}

bool ZuptDetector::update(Vec3 accel, Vec3 gyro, float speed)
{
  accelBuffer_[0][index_] = accel.x;
  accelBuffer_[1][index_] = accel.y;
  accelBuffer_[2][index_] = accel.z;
  gyroBuffer_[0][index_] = gyro.x;
  gyroBuffer_[1][index_] = gyro.y;
  gyroBuffer_[2][index_] = gyro.z;

  index_ = (index_ + 1) % ZUPT_WINDOW_SIZE;
  if (index_ == 0)
  {
    filled_ = true;
  }
  if (!filled_)
  {
    stationary_ = false;
    lowVariance_ = false;
    lowMean_ = false;
    return false;
  }

  float accelMean[3], gyroMean[3], var;
  accelVariance_ = 0.0f;
  gyroVariance_ = 0.0f;
  for (int a = 0; a < 3; a++)
  {
    meanAndVariance(accelBuffer_[a], ZUPT_WINDOW_SIZE, accelMean[a], var);
    accelVariance_ += var;
    meanAndVariance(gyroBuffer_[a], ZUPT_WINDOW_SIZE, gyroMean[a], var);
    gyroVariance_ += var;
  }
  accelMeanNorm_ = sqrtf(accelMean[0] * accelMean[0] + accelMean[1] * accelMean[1] + accelMean[2] * accelMean[2]);
  gyroMeanNorm_ = sqrtf(gyroMean[0] * gyroMean[0] + gyroMean[1] * gyroMean[1] + gyroMean[2] * gyroMean[2]);

  lowVariance_ = accelVariance_ < ZUPT_ACCEL_VARIANCE_THRESHOLD && gyroVariance_ < ZUPT_GYRO_VARIANCE_THRESHOLD;
  lowMean_ = accelMeanNorm_ < ZUPT_ACCEL_MEAN_THRESHOLD && gyroMeanNorm_ < ZUPT_GYRO_MEAN_THRESHOLD;

  if (lowVariance_ && lowMean_)
  {
    stillCounter_++;
  }
  else
  {
    stillCounter_ = 0;
  }
  const int required = (speed > ZUPT_SLOW_CONFIRM_SPEED) ? ZUPT_SLOW_CONFIRM_SAMPLES : ZUPT_REQUIRED_SAMPLES;
  stationary_ = stillCounter_ > required;
  return stationary_;
}
