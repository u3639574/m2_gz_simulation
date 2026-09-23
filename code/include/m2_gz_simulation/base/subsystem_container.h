#ifndef M2_GZ_SIMULATION_BASE_SUBSYSTEM_CONTAINER_H
#define M2_GZ_SIMULATION_BASE_SUBSYSTEM_CONTAINER_H

#include "m2_fsm_cpp/base/fsm_define.h"

namespace m2::gz_simulation {

struct subsystem_container_t {
    fsm::subsystem_instance_id_t gripper_subsystem{
        fsm::SUBSYSTEM_INSTANCE_ID_INVALID};
    fsm::subsystem_instance_id_t lift_subsystem{
        fsm::SUBSYSTEM_INSTANCE_ID_INVALID};

    // Add more Gazebo-simulation subsystems here.
};

inline subsystem_container_t g_subsystem_container;

}  // namespace m2::gz_simulation

#endif  // M2_GZ_SIMULATION_BASE_SUBSYSTEM_CONTAINER_H