#include "gaussian_filter.h"
#include <math.h>

CausalGaussianFilter::CausalGaussianFilter()
{
  float sum = 0.0f;
  for (int i = 0; i < GAUSSIAN_LIST_SIZE; i++)
  {
    kernel_[i] = expf(-(float)(i * i) / (2.0f * GAUSSIAN_SIGMA * GAUSSIAN_SIGMA));
    sum += kernel_[i];
  }
  for (int i = 0; i < GAUSSIAN_LIST_SIZE; i++)
  {
    kernel_[i] /= sum;
  }
  reset();
}

void CausalGaussianFilter::reset()
{
  index_ = 0;
  primed_ = false;
  for (int i = 0; i < GAUSSIAN_LIST_SIZE; i++)
  {
    buffer_[i] = 0.0f;
  }
}

float CausalGaussianFilter::update(float x)
{
  if (!primed_)
  {
    for (int i = 0; i < GAUSSIAN_LIST_SIZE; i++)
    {
      buffer_[i] = x;
    }
    primed_ = true;
  }
  index_ = (index_ + 1) % GAUSSIAN_LIST_SIZE;
  buffer_[index_] = x;

  float y = 0.0f;
  for (int i = 0; i < GAUSSIAN_LIST_SIZE; i++)
  {
    const int idx = (index_ - i + GAUSSIAN_LIST_SIZE) % GAUSSIAN_LIST_SIZE;
    y += kernel_[i] * buffer_[idx];
  }
  return y;
}
