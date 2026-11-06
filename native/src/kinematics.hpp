#pragma once
#include <cstddef>

namespace cadence {
struct MotionLimits {
  double min_linear_velocity;
  double max_linear_velocity;
  double min_angular_velocity;
  double max_angular_velocity;
  double max_linear_acceleration;
  double max_angular_acceleration;
};

// Output buffers must not alias inputs; the adapter validates buffer lengths, values, and limits.
void integrate_batch(const double *states, const double *previous_velocities,
  const double *commands, std::size_t count, double dt, const MotionLimits &limits,
  double *next_states, double *velocities);
}
