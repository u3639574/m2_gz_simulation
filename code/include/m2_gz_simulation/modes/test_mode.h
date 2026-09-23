#ifndef M2_GZ_SIMULATION_MODES_TEST_MODE_H
#define M2_GZ_SIMULATION_MODES_TEST_MODE_H

#include "m2_fsm_cpp/robots/ros2/fsm_ros2_ps_mode.h"
#include "m2_gz_simulation/base/gz_simulation_define.h"

namespace m2::gz_simulation {

class test_mode : public fsm::input::ps_mode {
public:
    explicit test_mode() noexcept;

    void tick() noexcept override;
    void on_enter() noexcept override;
    void on_exit() noexcept override;
    void on_input(const ps_input_data_type& data) noexcept override;
};

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_MODES_TEST_MODE_H