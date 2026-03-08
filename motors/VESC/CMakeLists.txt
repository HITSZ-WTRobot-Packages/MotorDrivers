add_library(MotorDriversVESC STATIC
    "./vesc.cpp"
)

target_include_directories(MotorDriversVESC
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(MotorDriversVESC PUBLIC MotorDrivers::Core)
target_link_libraries(MotorDriversVESC PUBLIC bsp::CANDriver)
target_link_libraries(MotorDriversVESC PUBLIC services::Watchdog)
target_link_libraries(MotorDriversVESC PUBLIC libs::FixedMap)

# alias for external use
add_library(MotorDrivers::VESC ALIAS MotorDriversVESC)
