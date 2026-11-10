#pragma once
#include "m2_fsm_cpp/commands/composite/fsm_sequential_command_group.h"
namespace m2::gz_simulation {
// Nezha move_to_staff_rack structure: named action -> configured PID pose command.
class move_to_earth_block_command : public fsm::sequential_command_group {
public: move_to_earth_block_command() noexcept;
};
class move_to_nearest_slope_command : public fsm::sequential_command_group {
public: explicit move_to_nearest_slope_command(const std::string& side = "") noexcept;
};
class move_up_slope_command : public fsm::sequential_command_group {
public: explicit move_up_slope_command(const std::string& side = "") noexcept;
};
}  // namespace m2::gz_simulation
