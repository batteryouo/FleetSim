#include "godot_lidar.hpp"
#include "native_frames.hpp"
#include <cmath>
#include <limits>
#include <godot_cpp/classes/box_shape3d.hpp>
#include <godot_cpp/classes/cylinder_shape3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

namespace cadence {
using namespace godot;
namespace {
constexpr uint32_t LIDAR_LAYER = 1;
bool number(const Variant &value, double &out) {
  if (value.get_type() != Variant::FLOAT && value.get_type() != Variant::INT) {
    return false;
  }
  out = static_cast<double>(value);
  return std::isfinite(out);
}
bool field(const Dictionary &value, const char *key, double &out) {
  return number(value.get(key, Variant()), out);
}
bool pair(const Variant &value, double &x, double &y) {
  if (value.get_type() != Variant::ARRAY) {
    return false;
  }
  const Array array = value;
  return array.size() == 2 && number(array[0], x) && number(array[1], y);
}
struct Wall {
  double x, y, width, depth, yaw, lo, hi;
};
}

void CadenceLidar::_bind_methods() {
  ClassDB::bind_method(D_METHOD("configure", "map", "robot", "frame"), &CadenceLidar::configure);
  ClassDB::bind_method(D_METHOD("scan_batch", "snapshot"), &CadenceLidar::scan_batch);
  ClassDB::bind_method(D_METHOD("close"), &CadenceLidar::close);
}
int CadenceLidar::configure(const Dictionary &map, const Dictionary &robot, const Basis &frame) {
  close();
  auto invalid = []() {
    UtilityFunctions::push_error("Lidar requires a valid compiled REP-103 map, robot spec, and orthonormal frame");
    return ERR_INVALID_PARAMETER;
  };
  if (map.get("frame", "") != Variant("rep103") || map.get("primitives", Variant()).get_type() != Variant::ARRAY
      || robot.get("body", Variant()).get_type() != Variant::DICTIONARY
      || robot.get("footprint", Variant()).get_type() != Variant::DICTIONARY
      || robot.get("lidar", Variant()).get_type() != Variant::DICTIONARY
      || !frame.is_finite() || !frame.is_equal_approx(frame.orthonormalized()) || std::abs(frame.determinant() - 1) > 1e-5) {
    return invalid();
  }
  const Dictionary body = robot["body"], footprint = robot["footprint"], lidar = robot["lidar"];
  if (lidar.get("mount", Variant()).get_type() != Variant::DICTIONARY
      || footprint.get("center", Variant()).get_type() != Variant::DICTIONARY) {
    return invalid();
  }
  const Dictionary mount = lidar["mount"], center = footprint["center"];
  double lo, hi, radius, mount_yaw, beam_count;
  if (!field(body, "z_min", lo) || !field(body, "z_max", hi) || hi <= lo
      || !field(footprint, "radius", radius) || radius <= 0
      || !field(center, "x", center_x_) || !field(center, "y", center_y_)
      || !field(mount, "x", mount_x_) || !field(mount, "y", mount_y_) || !field(mount, "z", mount_z_)
      || !field(mount, "yaw", mount_yaw) || !field(lidar, "beam_count", beam_count)
      || beam_count < 2 || beam_count != std::floor(beam_count) || beam_count > std::numeric_limits<int32_t>::max()
      || !field(lidar, "angle_min", angle_min_) || !field(lidar, "angle_max", angle_max_) || angle_max_ <= angle_min_
      || !field(lidar, "range_min", range_min_) || !field(lidar, "range_max", range_max_)
      || range_min_ < 0 || range_max_ <= range_min_) {
    return invalid();
  }
  std::vector<Wall> walls;
  const Array primitives = map["primitives"];
  for (int64_t index = 0; index < primitives.size(); ++index) {
    if (primitives[index].get_type() != Variant::DICTIONARY) {
      return invalid();
    }
    const Dictionary primitive = primitives[index];
    Wall wall;
    if (!pair(primitive.get("center", Variant()), wall.x, wall.y)
        || !pair(primitive.get("size", Variant()), wall.width, wall.depth) || wall.width <= 0 || wall.depth <= 0
        || !field(primitive, "yaw", wall.yaw) || !field(primitive, "z_min", wall.lo)
        || !field(primitive, "z_max", wall.hi) || wall.hi <= wall.lo) {
      return invalid();
    }
    walls.push_back(wall);
  }
  frame_ = frame;
  spec_ = robot.duplicate(true);
  body_z_ = lo + (hi - lo) / 2;
  increment_ = (angle_max_ - angle_min_) / (beam_count - 1);
  for (int64_t beam = 0; beam < static_cast<int64_t>(beam_count); ++beam) {
    const double angle = angle_min_ + beam * increment_ + mount_yaw;
    cosines_.push_back(std::cos(angle));
    sines_.push_back(std::sin(angle));
  }
  auto *server = PhysicsServer3D::get_singleton();
  space_ = server->space_create();
  server->space_set_active(space_, true);
  Ref<CylinderShape3D> cylinder;
  cylinder.instantiate();
  cylinder->set_radius(radius);
  cylinder->set_height(hi - lo);
  cylinder->set_margin(0);
  robot_shape_ = cylinder;
  for (const auto &wall : walls) {
    Ref<BoxShape3D> shape;
    shape.instantiate();
    shape->set_size(frames::size_to_godot(frame_, wall.width, wall.depth, wall.hi - wall.lo));
    shape->set_margin(0);
    shapes_.push_back(shape);
    const Transform3D transform = frames::wall_to_godot(frame_, wall.x, wall.y, wall.lo + (wall.hi - wall.lo) / 2, wall.yaw);
    walls_.push_back(create_body(shape->get_rid(), transform));
  }
  return OK;
}
RID CadenceLidar::create_body(RID shape, const Transform3D &transform) {
  auto *server = PhysicsServer3D::get_singleton();
  const RID body = server->body_create();
  server->body_set_mode(body, PhysicsServer3D::BODY_MODE_STATIC);
  server->body_set_collision_layer(body, LIDAR_LAYER);
  server->body_set_collision_mask(body, 0);
  server->body_add_shape(body, shape);
  server->body_set_state(body, PhysicsServer3D::BODY_STATE_TRANSFORM, transform);
  server->body_set_space(body, space_);
  return body;
}
TypedArray<Dictionary> CadenceLidar::scan_batch(const Dictionary &snapshot) {
  if (!space_.is_valid() || snapshot.get("robot_spec", Variant()) != Variant(spec_)
      || snapshot.get("ids", Variant()).get_type() != Variant::ARRAY
      || snapshot.get("poses", Variant()).get_type() != Variant::PACKED_FLOAT64_ARRAY) {
    UtilityFunctions::push_error("Lidar snapshot must match the configured robot spec and use float64 poses");
    return {};
  }
  const Array ids = snapshot["ids"];
  const PackedFloat64Array poses = snapshot["poses"];
  if (poses.size() != ids.size() * 3) {
    UtilityFunctions::push_error("Lidar requires three pose values per robot");
    return {};
  }
  Dictionary seen;
  for (int64_t index = 0; index < ids.size(); ++index) {
    if (ids[index].get_type() != Variant::STRING || seen.has(ids[index])) {
      UtilityFunctions::push_error("Lidar robot ids must be unique strings");
      return {};
    }
    seen[ids[index]] = true;
  }
  const double *state = poses.ptr();
  for (int64_t index = 0; index < poses.size(); ++index) {
    if (!std::isfinite(state[index])) {
      UtilityFunctions::push_error("Lidar poses must be finite");
      return {};
    }
  }
  auto *server = PhysicsServer3D::get_singleton();
  auto *direct = server->space_get_direct_state(space_);
  if (!direct) {
    UtilityFunctions::push_error("Lidar must run in a physics-safe execution context");
    return {};
  }
  const Array previous = robots_.keys();
  for (int64_t index = 0; index < previous.size(); ++index) {
    if (!seen.has(previous[index])) {
      server->free_rid(robots_[previous[index]]);
      robots_.erase(previous[index]);
    }
  }
  // Static transforms are immediate for teleported query proxies; kinematic updates wait for a physics step.
  for (int64_t index = 0; index < ids.size(); ++index) {
    const double c = std::cos(state[index * 3 + 2]), s = std::sin(state[index * 3 + 2]);
    const double x = state[index * 3] + c * center_x_ - s * center_y_;
    const double y = state[index * 3 + 1] + s * center_x_ + c * center_y_;
    const Transform3D transform(Basis(), frames::point_to_godot(frame_, x, y, body_z_));
    if (!robots_.has(ids[index])) {
      robots_[ids[index]] = create_body(robot_shape_->get_rid(), transform);
    } else {
      server->body_set_state(robots_[ids[index]], PhysicsServer3D::BODY_STATE_TRANSFORM, transform);
    }
  }
  TypedArray<Dictionary> scans;
  for (int64_t index = 0; index < ids.size(); ++index) {
    const double c = std::cos(state[index * 3 + 2]), s = std::sin(state[index * 3 + 2]);
    const double x = state[index * 3] + c * mount_x_ - s * mount_y_;
    const double y = state[index * 3 + 1] + s * mount_x_ + c * mount_y_;
    const Vector3 origin = frames::point_to_godot(frame_, x, y, mount_z_);
    Ref<PhysicsRayQueryParameters3D> query;
    query.instantiate();
    query->set_from(origin);
    TypedArray<RID> excluded;
    excluded.push_back(robots_[ids[index]]);
    query->set_exclude(excluded);
    query->set_collision_mask(LIDAR_LAYER);
    query->set_hit_from_inside(true);
    PackedFloat64Array ranges;
    ranges.resize(static_cast<int64_t>(cosines_.size()));
    double *output = ranges.ptrw();
    for (std::size_t beam = 0; beam < cosines_.size(); ++beam) {
      const double end_x = x + range_max_ * (c * cosines_[beam] - s * sines_[beam]);
      const double end_y = y + range_max_ * (s * cosines_[beam] + c * sines_[beam]);
      query->set_to(frames::point_to_godot(frame_, end_x, end_y, mount_z_));
      const Dictionary hit = direct->intersect_ray(query);
      const double distance = hit.is_empty() ? std::numeric_limits<double>::infinity()
        : static_cast<double>(origin.distance_to(static_cast<Vector3>(hit["position"])));
      output[beam] = distance < range_min_ ? -std::numeric_limits<double>::infinity()
        : (distance > range_max_ ? std::numeric_limits<double>::infinity() : distance);
    }
    Dictionary scan;
    scan["robot_id"] = ids[index];
    scan["angle_min"] = angle_min_;
    scan["angle_max"] = angle_max_;
    scan["angle_increment"] = increment_;
    scan["range_min"] = range_min_;
    scan["range_max"] = range_max_;
    scan["ranges"] = ranges;
    scans.push_back(scan);
  }
  return scans;
}
void CadenceLidar::close() {
  auto *server = PhysicsServer3D::get_singleton();
  for (RID body : walls_) {
    server->free_rid(body);
  }
  const Array bodies = robots_.values();
  for (int64_t index = 0; index < bodies.size(); ++index) {
    server->free_rid(bodies[index]);
  }
  walls_.clear();
  robots_.clear();
  shapes_.clear();
  robot_shape_.unref();
  cosines_.clear();
  sines_.clear();
  spec_.clear();
  if (space_.is_valid()) {
    server->free_rid(space_);
    space_ = RID();
  }
}
CadenceLidar::~CadenceLidar() {
  close();
}
}
