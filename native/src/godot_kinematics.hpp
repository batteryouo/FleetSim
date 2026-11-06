#pragma once
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>

namespace cadence {
class CadenceKinematics : public godot::RefCounted {
  GDCLASS(CadenceKinematics, godot::RefCounted)
protected:
  static void _bind_methods();
public:
  godot::Dictionary advance(const godot::PackedFloat64Array &states,
    const godot::PackedFloat64Array &previous_velocities,
    const godot::PackedFloat64Array &commands, double dt, const godot::Dictionary &limits) const;
};
}
