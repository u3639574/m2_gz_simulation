m2_gz_simulation_set_selectors(m2-gz-simulation)
m2_gz_simulation_ros2_add_object_library(simulation-control
    src/control/joint_feedback.cpp
    src/subsystems/lift_subsystem.cpp
    src/subsystems/gripper_subsystem.cpp
    src/commands/position_command.cpp
    src/base/simulation_robot.cpp
)
set_property(TARGET simulation-control-objects PROPERTY M2_GZ_SIMULATION_SELECTOR m2-gz-simulation)

m2_gz_simulation_set_selectors(fsm_node)
m2_gz_simulation_ros2_add_object_library(simulation-node src/nodes/fsm_node.cpp)
set_property(TARGET simulation-node-objects PROPERTY M2_GZ_SIMULATION_SELECTOR fsm_node)
