include_guard(GLOBAL)

m2_gz_simulation_add_interface()

m2_gz_simulation_ros2_add_library(m2-gz-simulation
    SELECTORS
        m2-gz-simulation
)

m2b_install_targets(m2-gz-simulation m2-gz-simulation-interface)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
    PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY config model docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
