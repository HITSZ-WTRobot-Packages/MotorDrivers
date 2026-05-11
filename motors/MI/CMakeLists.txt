add_library(MotorDriversMIMotor STATIC
    "./mi.cpp"
)

target_include_directories(MotorDriversMIMotor
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(MotorDriversMIMotor PUBLIC MotorDrivers::Core)
target_link_libraries(MotorDriversMIMotor PUBLIC bsp::CANDriver)
target_link_libraries(MotorDriversMIMotor PUBLIC services::Watchdog)
target_link_libraries(MotorDriversMIMotor PUBLIC libs::FixedMap)

add_library(MotorDrivers::MIMotor ALIAS MotorDriversMIMotor)
