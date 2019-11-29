#include "filter.h"

#include "math.h"
#include "project.h"

void iir_filter_lpf2_set_freq(iir_filter_lpf2 *filter, float sample_freq, float cutoff_freq) {
  const float fr = sample_freq / cutoff_freq;
  const float ohm = tanf(M_PI_F / fr);
  const float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;

  filter->cutoff_freq = cutoff_freq;

  if (filter->cutoff_freq > 0.0f) {
    filter->b0 = ohm * ohm / c;
    filter->b1 = 2.0f * filter->b0;
    filter->b2 = filter->b0;
    filter->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    filter->a2 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
  }
}

float iir_filter_lpf2_apply(iir_filter_lpf2 *filter, float sample) {
  if (filter->cutoff_freq <= 0.0f) {
    return sample; /* No filtering */
  }

  float delay_element_0 = sample - filter->delay_element_1 * filter->a1 - filter->delay_element_2 * filter->a2;
  /* Do the filtering */
  if (isnan(delay_element_0) || isinf(delay_element_0)) {
    /* Don't allow bad values to propogate via the filter */
    delay_element_0 = sample;
  }

  const float output = delay_element_0 * filter->b0 + filter->delay_element_1 * filter->b1 + filter->delay_element_2 * filter->b2;
  filter->delay_element_2 = filter->delay_element_1;
  filter->delay_element_1 = delay_element_0;

  /* Return the value.  Should be no need to check limits */
  return output;
}
