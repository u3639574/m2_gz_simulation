cmake_minimum_required(VERSION 3.16)
project(m2_gz_simulation LANGUAGES C CXX VERSION 1.0.0)
include(../m2b/cmake/modules/m2b_setup.cmake)

m2b_begin()

m2b_add_subdirectory(code)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME world_contract_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/world_contract_test.py)
    ament_add_gtest(keyboard_command_test test/keyboard_command_test.cpp)
    target_include_directories(keyboard_command_test PRIVATE code/include)
    target_link_libraries(keyboard_command_test m2-gz-simulation
        m2_fsm_cpp::m2-fsm-cpp m2_fsm_cpp::m2-fsm-cpp-interface
        trolly::trolly-interface trolly::trolly-ros2)
    ament_target_dependencies(keyboard_command_test rclcpp sensor_msgs std_msgs std_srvs ros_gz_interfaces)
    ament_add_gtest(control_test test/control_test.cpp)
    target_include_directories(control_test PRIVATE code/include)
    target_link_libraries(control_test m2-gz-simulation
        m2_fsm_cpp::m2-fsm-cpp m2_fsm_cpp::m2-fsm-cpp-interface
        trolly::trolly-interface trolly::trolly-ros2)
    ament_target_dependencies(control_test rclcpp sensor_msgs std_msgs std_srvs ros_gz_interfaces)
endif()

m2b_end()
