#pragma once
#include <map>
#include "geometry_msgs/msg/twist.hpp"
#include "m2_gz_simulation/base/keyboard_key.h"
#include "m2_fsm_cpp/base/fsm_define.h"

namespace m2::gz_simulation {
// Keyboard mode lifecycle mirrors the real robot's modes, without PS-specific input/LEDs.
class test_mode {
public:
    void tick() noexcept;
    void on_enter() noexcept;
    void on_exit() noexcept;
    void on_input(keyboard_key key) noexcept;
private:
    std::map<keyboard_key, fsm::command_instance_id_t> active_commands_;
    geometry_msgs::msg::Twist keyboard_velocity_;
    bool manual_active_{false};
    double last_tick_time_{-1.0};
};
}  // namespace m2::gz_simulation
