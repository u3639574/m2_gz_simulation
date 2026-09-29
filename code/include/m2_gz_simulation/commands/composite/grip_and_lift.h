#pragma once
#include "m2_fsm_cpp/commands/composite/fsm_sequential_command_group.h"
#include "m2_fsm_cpp/commands/composite/fsm_parallel_command_group.h"

namespace m2::gz_simulation {
class close_then_raise_command : public fsm::sequential_command_group {
public:
    close_then_raise_command() noexcept;
};
class open_and_lower_command : public fsm::parallel_command_group {
public:
    open_and_lower_command() noexcept;
};
}  // namespace m2::gz_simulation
