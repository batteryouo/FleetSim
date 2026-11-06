#include "kinematics.hpp"
#include <algorithm>
#include <cmath>

namespace cadence {
namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double TAU = 2.0 * PI;

double sinc(double angle) {
  if (std::abs(angle) < 0.0001) {
    const double squared = angle * angle;
    return 1.0 - squared / 6.0 + squared * squared / 120.0 - squared * squared * squared / 5040.0;
  }
  return std::sin(angle) / angle;
}
double normalize_yaw(double angle) {
  double yaw = std::fmod(angle, TAU);
  if (yaw <= -PI) {
    yaw += TAU;
  } else if (yaw > PI) {
    yaw -= TAU;
  }
  return yaw;
}
}

void integrate_batch(const double *states, const double *previous_velocities,
  const double *commands, std::size_t count, double dt, const MotionLimits &limits,
  double *next_states, double *velocities) {
  const double linear_delta = limits.max_linear_acceleration * dt;
  const double angular_delta = limits.max_angular_acceleration * dt;
  for (std::size_t robot = 0; robot < count; ++robot) {
    const std::size_t pose_index = robot * 3;
    const std::size_t velocity_index = robot * 2;
    double v = previous_velocities[velocity_index] + std::clamp(
      commands[velocity_index] - previous_velocities[velocity_index], -linear_delta, linear_delta);
    double w = previous_velocities[velocity_index + 1] + std::clamp(
      commands[velocity_index + 1] - previous_velocities[velocity_index + 1], -angular_delta, angular_delta);
    v = std::clamp(v, limits.min_linear_velocity, limits.max_linear_velocity);
    w = std::clamp(w, limits.min_angular_velocity, limits.max_angular_velocity);
    const double half_turn = w * dt / 2.0;
    const double distance = v * dt * sinc(half_turn);
    const double heading = states[pose_index + 2] + half_turn;
    next_states[pose_index] = states[pose_index] + distance * std::cos(heading);
    next_states[pose_index + 1] = states[pose_index + 1] + distance * std::sin(heading);
    next_states[pose_index + 2] = normalize_yaw(states[pose_index + 2] + w * dt);
    velocities[velocity_index] = v;
    velocities[velocity_index + 1] = w;
  }
}
}
