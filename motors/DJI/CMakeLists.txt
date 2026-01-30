add_library(MotorDriversDJI STATIC
    "./dji.cpp"
)

target_include_directories(MotorDriversDJI
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(MotorDriversDJI PUBLIC MotorDrivers::Core)
target_link_libraries(MotorDriversDJI PUBLIC bsp::CANDriver)
target_link_libraries(MotorDriversDJI PUBLIC services::Watchdog)

# alias for external use
add_library(MotorDrivers::DJI ALIAS MotorDriversDJI)
