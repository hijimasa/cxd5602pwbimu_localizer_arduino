#include "test_framework.h"
#include "localizer/gaussian_filter.h"

TEST(gaussian_kernel_is_normalised_and_decreasing)
{
  CausalGaussianFilter f;
  float sum = 0.0f;
  for (int i = 0; i < GAUSSIAN_LIST_SIZE; i++)
  {
    sum += f.kernel(i);
    if (i > 0) CHECK_LT(f.kernel(i), f.kernel(i - 1));
  }
  CHECK_NEAR(sum, 1.0f, 1e-5);
}

TEST(gaussian_unity_dc_gain_without_start_transient)
{
  CausalGaussianFilter f;
  for (int i = 0; i < 100; i++)
  {
    CHECK_NEAR(f.update(3.5f), 3.5f, 1e-5);
  }
}

TEST(gaussian_impulse_response_equals_kernel)
{
  // The current sample must receive kernel[0] (the largest weight), the
  // sample n steps back kernel[n].
  CausalGaussianFilter f;
  for (int i = 0; i < GAUSSIAN_LIST_SIZE; i++) f.update(0.0f);
  const float y0 = f.update(1.0f);
  CHECK_NEAR(y0, f.kernel(0), 1e-6);
  for (int n = 1; n < GAUSSIAN_LIST_SIZE; n++)
  {
    CHECK_NEAR(f.update(0.0f), f.kernel(n), 1e-6);
  }
  CHECK_NEAR(f.update(0.0f), 0.0f, 1e-6);
}

TEST(gaussian_reset_reprimes)
{
  CausalGaussianFilter f;
  f.update(10.0f);
  f.reset();
  CHECK_NEAR(f.update(-2.0f), -2.0f, 1e-6);
}
