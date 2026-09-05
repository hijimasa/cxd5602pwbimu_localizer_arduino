/**
 * @file gaussian_filter.h
 * @brief Causal (one-sided) Gaussian FIR filter.
 *
 * kernel[i] weights the sample i steps in the past; the current sample gets
 * kernel[0]. The buffer is primed with the first sample to avoid a start-up
 * transient.
 */
#ifndef GAUSSIAN_FILTER_H
#define GAUSSIAN_FILTER_H

#include "localizer_config.h"

class CausalGaussianFilter
{
public:
  CausalGaussianFilter();

  void reset();

  /** Push one sample and return the filtered value. */
  float update(float x);

  /** Normalised kernel weight for lag i (for tests / diagnostics). */
  float kernel(int i) const { return kernel_[i]; }

private:
  float kernel_[GAUSSIAN_LIST_SIZE];
  float buffer_[GAUSSIAN_LIST_SIZE];
  int index_; // position of the most recent sample
  bool primed_;
};

#endif // GAUSSIAN_FILTER_H
