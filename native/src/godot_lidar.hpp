#pragma once
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/shape3d.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <vector>

namespace cadence {
class CadenceLidar : public godot::RefCounted {
  GDCLASS(CadenceLidar, godot::RefCounted)
  godot::Dictionary spec_;
  godot::Basis frame_;
  godot::RID space_;
  godot::Dictionary robots_;
  std::vector<godot::RID> walls_;
  std::vector<godot::Ref<godot::Shape3D>> shapes_;
  godot::Ref<godot::Shape3D> robot_shape_;
  std::vector<double> cosines_, sines_;
  double mount_x_ = 0, mount_y_ = 0, mount_z_ = 0, body_z_ = 0, center_x_ = 0, center_y_ = 0;
  double angle_min_ = 0, angle_max_ = 0, increment_ = 0, range_min_ = 0, range_max_ = 0;
  godot::RID create_body(godot::RID shape, const godot::Transform3D &transform);
protected:
  static void _bind_methods();
public:
  int configure(const godot::Dictionary &map, const godot::Dictionary &robot, const godot::Basis &frame);
  godot::TypedArray<godot::Dictionary> scan_batch(const godot::Dictionary &snapshot);
  void close();
  ~CadenceLidar() override;
};
}
