add_library(MotorDriversMIMotor STATIC
    "./mi.cpp"
)

target_include_directories(MotorDriversMIMotor
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(MotorDriversMIMotor PUBLIC MotorDrivers::Core)
target_link_libraries(MotorDriversMIMotor PUBLIC bsp::CANDriver)

# alias for external use
add_library(MotorDrivers::MIMotor ALIAS MotorDriversMIMotor)
