#include "godot_kinematics.hpp"
#include "godot_lidar.hpp"
#include <gdextension_interface.h>
#include <godot_cpp/godot.hpp>

namespace {
void initialize(godot::ModuleInitializationLevel level) {
  if (level == godot::MODULE_INITIALIZATION_LEVEL_SCENE) {
    godot::ClassDB::register_class<cadence::CadenceKinematics>();
    godot::ClassDB::register_class<cadence::CadenceLidar>();
  }
}
void uninitialize(godot::ModuleInitializationLevel) {}
}

extern "C" GDExtensionBool GDE_EXPORT cadence_library_init(
  GDExtensionInterfaceGetProcAddress get_proc_address,
  GDExtensionClassLibraryPtr library, GDExtensionInitialization *initialization) {
  godot::GDExtensionBinding::InitObject binding(get_proc_address, library, initialization);
  binding.register_initializer(initialize);
  binding.register_terminator(uninitialize);
  binding.set_minimum_library_initialization_level(godot::MODULE_INITIALIZATION_LEVEL_SCENE);
  return binding.init();
}
