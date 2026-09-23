m2_gz_simulation_ros2_add_executable(fsm_node
    SELECTORS
        fsm_node
)

m2_gz_simulation_ros2_add_executable(control_node
    SELECTORS
        control_node
)

m2b_install_targets(fsm_node control_node)

m2b_ros2_cmake_package(
    EXPORT_DEPENDS
        rclcpp std_msgs std_srvs sensor_msgs m2_interfaces trolly m2_fsm_cpp
)