#pragma once
#include "m2_gz_simulation/commands/lift/set_lift_pos.h"
namespace m2::gz_simulation {
enum class lift_endpoint { TOP, BOTTOM };
class move_lift_command : public set_lift_pos_command {
public:
    explicit move_lift_command(lift_endpoint endpoint) noexcept;
};
class raise_lift_command : public move_lift_command {
public: raise_lift_command() noexcept : move_lift_command(lift_endpoint::TOP) {}
};
class lower_lift_command : public move_lift_command {
public: lower_lift_command() noexcept : move_lift_command(lift_endpoint::BOTTOM) {}
};
}  // namespace m2::gz_simulation
