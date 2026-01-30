add_library(MotorDriversController STATIC
    "./motor_pos_controller.cpp"
    "./motor_vel_controller.cpp"
)

target_include_directories(MotorDriversController
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(MotorDriversController PUBLIC MotorDrivers::Core)
target_link_libraries(MotorDriversController PUBLIC libs::PIDMotor)

# alias for external use
add_library(MotorDrivers::Controller ALIAS MotorDriversController)
