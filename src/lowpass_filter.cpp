// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/lowpass_filter.h>

#include <cmath>

namespace franka {

double lowpassFilter(double sample_time, double y, double y_last, double cutoff_frequency) {
  double gain = sample_time / (sample_time + (1.0 / (2.0 * M_PI * cutoff_frequency)));
  return gain * y + (1 - gain) * y_last;
}

}  // namespace franka
