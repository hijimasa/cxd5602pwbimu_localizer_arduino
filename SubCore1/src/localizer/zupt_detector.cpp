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
  accelVariance_ = 0.0f;
  gyroVariance_ = 0.0f;
}

float ZuptDetector::variance(const float *buffer, int size)
{
  float mean = 0.0f;
  for (int i = 0; i < size; i++)
  {
    mean += buffer[i];
  }
  mean /= size;
  float var = 0.0f;
  for (int i = 0; i < size; i++)
  {
    const float d = buffer[i] - mean;
    var += d * d;
  }
  return var / size;
}

bool ZuptDetector::update(Vec3 accel, Vec3 gyro)
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
    return false;
  }

  accelVariance_ = variance(accelBuffer_[0], ZUPT_WINDOW_SIZE) +
                   variance(accelBuffer_[1], ZUPT_WINDOW_SIZE) +
                   variance(accelBuffer_[2], ZUPT_WINDOW_SIZE);
  gyroVariance_ = variance(gyroBuffer_[0], ZUPT_WINDOW_SIZE) +
                  variance(gyroBuffer_[1], ZUPT_WINDOW_SIZE) +
                  variance(gyroBuffer_[2], ZUPT_WINDOW_SIZE);

  if (accelVariance_ < ZUPT_ACCEL_VARIANCE_THRESHOLD && gyroVariance_ < ZUPT_GYRO_VARIANCE_THRESHOLD)
  {
    stillCounter_++;
  }
  else
  {
    stillCounter_ = 0;
  }
  stationary_ = stillCounter_ > ZUPT_REQUIRED_SAMPLES;
  return stationary_;
}
