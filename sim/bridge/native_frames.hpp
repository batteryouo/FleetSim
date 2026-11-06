#pragma once
#include <godot_cpp/variant/transform3d.hpp>

namespace cadence::frames {
inline godot::Vector3 point_to_godot(const godot::Basis &frame, double x, double y, double z) {
  return frame.xform(godot::Vector3(x, y, z));
}
inline godot::Vector3 size_to_godot(const godot::Basis &frame, double x, double y, double z) {
  return frame.xform(godot::Vector3(x, y, z)).abs();
}
inline godot::Transform3D wall_to_godot(const godot::Basis &frame, double x, double y, double z, double yaw) {
  return godot::Transform3D(godot::Basis(frame.get_column(2), yaw), point_to_godot(frame, x, y, z));
}
}
