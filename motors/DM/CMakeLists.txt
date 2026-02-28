add_library(MotorDriversDM STATIC
    "./dm.cpp"
)

target_include_directories(MotorDriversDM
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# link dependencies if any
target_link_libraries(MotorDriversDM PUBLIC MotorDrivers::Core)
target_link_libraries(MotorDriversDM PUBLIC bsp::CANDriver)
target_link_libraries(MotorDriversDM PUBLIC services::Watchdog)
target_link_libraries(MotorDriversDM PUBLIC libs::FixedMap)

# alias for external use
add_library(MotorDrivers::DM ALIAS MotorDriversDM)
