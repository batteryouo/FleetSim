#include "godot_kinematics.hpp"
#include "kinematics.hpp"
#include <cmath>
#include <cstdint>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

namespace cadence {
namespace {
bool finite_buffer(const godot::PackedFloat64Array &values) {
  const double *data = values.ptr();
  for (int64_t index = 0; index < values.size(); ++index) {
    if (!std::isfinite(data[index])) {
      return false;
    }
  }
  return true;
}
bool read_number(const godot::Dictionary &limits, const char *name, double &output) {
  const godot::Variant value = limits.get(name, godot::Variant());
  if (value.get_type() != godot::Variant::FLOAT && value.get_type() != godot::Variant::INT) {
    return false;
  }
  output = static_cast<double>(value);
  return std::isfinite(output);
}
bool read_limits(const godot::Dictionary &source, MotionLimits &limits) {
  return read_number(source, "min_linear_velocity", limits.min_linear_velocity)
    && read_number(source, "max_linear_velocity", limits.max_linear_velocity)
    && read_number(source, "min_angular_velocity", limits.min_angular_velocity)
    && read_number(source, "max_angular_velocity", limits.max_angular_velocity)
    && read_number(source, "max_linear_acceleration", limits.max_linear_acceleration)
    && read_number(source, "max_angular_acceleration", limits.max_angular_acceleration)
    && limits.min_linear_velocity < limits.max_linear_velocity
    && limits.min_angular_velocity < limits.max_angular_velocity
    && limits.max_linear_acceleration > 0.0
    && limits.max_angular_acceleration > 0.0;
}
}

void CadenceKinematics::_bind_methods() {
  godot::ClassDB::bind_method(godot::D_METHOD("advance", "states", "previous_velocities", "commands", "dt", "limits"),
    &CadenceKinematics::advance);
}
godot::Dictionary CadenceKinematics::advance(const godot::PackedFloat64Array &states,
  const godot::PackedFloat64Array &previous_velocities,
  const godot::PackedFloat64Array &commands, double dt, const godot::Dictionary &source) const {
  if (!std::isfinite(dt) || dt <= 0.0 || states.size() % 3 != 0) {
    godot::UtilityFunctions::push_error("Kinematics requires finite positive dt and three state components per robot");
    return {};
  }
  const int64_t count = states.size() / 3;
  if (previous_velocities.size() != count * 2 || commands.size() != count * 2) {
    godot::UtilityFunctions::push_error("Kinematics requires two velocities and two commands per robot");
    return {};
  }
  if (!finite_buffer(states) || !finite_buffer(previous_velocities) || !finite_buffer(commands)) {
    godot::UtilityFunctions::push_error("Kinematics inputs must be finite");
    return {};
  }
  MotionLimits limits;
  if (!read_limits(source, limits)) {
    godot::UtilityFunctions::push_error("Kinematics requires finite ordered velocity limits and positive accelerations");
    return {};
  }
  godot::PackedFloat64Array next_states;
  godot::PackedFloat64Array velocities;
  next_states.resize(states.size());
  velocities.resize(previous_velocities.size());
  integrate_batch(states.ptr(), previous_velocities.ptr(), commands.ptr(), static_cast<std::size_t>(count),
    dt, limits, next_states.ptrw(), velocities.ptrw());
  godot::Dictionary result;
  result["states"] = next_states;
  result["velocities"] = velocities;
  return result;
}
}
