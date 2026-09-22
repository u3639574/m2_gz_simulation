include_guard(GLOBAL)

function(m2_gz_simulation_add_interface)
    m2b_add_interface(m2-gz-simulation-interface
        INCLUDES
            ${PROJECT_SOURCE_DIR}/code/include
    )
endfunction()
