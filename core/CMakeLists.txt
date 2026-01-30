add_library(__MotorDrivers_Core INTERFACE)

target_include_directories(__MotorDrivers_Core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# link dependencies if any


# alias for external use
add_library(MotorDrivers::Core ALIAS __MotorDrivers_Core)
