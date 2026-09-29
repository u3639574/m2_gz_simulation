#pragma once
#include <cstdint>

namespace m2::gz_simulation {
// Gazebo GUI /keyboard/keypress values, verified with gz topic -e.
// Letter key codes are uppercase even when the user types lowercase.
enum class keyboard_key : std::int32_t {
    Q = 81, A = 65, Z = 90,
    W = 87, S = 83, X = 88,
    E = 69, D = 68, C = 67,
    R = 82, F = 70, V = 86,
    UP = 16777235, DOWN = 16777237,
    LEFT = 16777234, RIGHT = 16777236,
};
}  // namespace m2::gz_simulation
