include_guard(GLOBAL)

m2b_define_selector(M2_GZ_SIMULATION_SELECTOR)

function(m2_gz_simulation_set_selectors selectors)
    m2b_set_selectors(M2_GZ_SIMULATION_SELECTOR ${selectors})
endfunction()
