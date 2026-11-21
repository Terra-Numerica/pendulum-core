cmake_minimum_required(VERSION 3.19.4)
project(pendule_pi VERSION 0.5.4 LANGUAGES CXX)

# Build Release by default : the control loop must run with same optimizations
# as the demonstrator (see README)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies : fatal error if one is missing
# (zmq et zmqpp are linked by name, without using find_package)
find_package(pigpio REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# Warnings on all targets. Optimization level comes from CMAKE_BUILD_TYPE
# (Release = -O3 -DNDEBUG).
add_compile_options(-Wall)

# code shared by demo and low_level_interface

add_library(pendule_pi
        src/pendule_pi/joystick.cpp
        src/pendule_pi/pigpio.cpp
        src/pendule_pi/switch.cpp
        src/pendule_pi/motor.cpp
        src/pendule_pi/encoder.cpp
        src/pendule_pi/pendule.cpp
        src/pendule_pi/pendule_cpp_server.cpp
        src/pendule_pi/pendule_cpp_client.cpp
        src/pendule_pi/pendulum_config.cpp
        src/pendule_pi/sockets_config.cpp
)

target_include_directories(pendule_pi PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# PUBLIC : every binary linked to pendule_pi inherits these dependencies
target_link_libraries(pendule_pi PUBLIC
        pigpio::pigpio
        Threads::Threads
        zmq
        zmqpp
        yaml-cpp::yaml-cpp
)

target_compile_definitions(pendule_pi PUBLIC
        PENDULE_PI_VERSION="${PROJECT_VERSION}"
        $<$<CONFIG:Debug>:PENDULE_PI_DEBUG_ENABLED>
)

# Command line parsing shared by the executables. Built once as a library : CLI11 is expensive to compile
# (header-only), so it is isolated in cli_parser.cpp and precompiled.
add_library(pendule_cli STATIC
        src/bin/cli_parser.cpp
        include/pendule_pi/cli_parser.hpp
)
target_include_directories(pendule_cli PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(pendule_cli PRIVATE
        PENDULE_PI_VERSION="${PROJECT_VERSION}"
)
target_precompile_headers(pendule_cli PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/CLI11/CLI11.hpp
)

###
### Binaries
###

# ----------------------------------------
add_executable(low_level_interface
        src/bin/low_level_interface.cpp
        include/pendule_pi/pendulum_config.hpp
        include/pendule_pi/sockets_config.hpp
)
target_link_libraries(low_level_interface PRIVATE
        pendule_pi
        pendule_cli
)

# ----------------------------------------
add_executable(demo
        src/bin/demo/demo.cpp
        src/bin/demo/demo_config.cpp
        include/pendule_pi/sockets_config.hpp
        include/pendule_pi/demo_config.hpp
)
target_link_libraries(demo PRIVATE
        pendule_pi
        pendule_cli
)

# ----------------------------------------
add_executable(reset_gpio
        src/bin/reset_gpio.cpp
)
target_link_libraries(reset_gpio
        pendule_pi
        pendule_cli
)

# ----------------------------------------
add_executable(check_encoders
        src/bin/check_encoders.cpp
)
target_link_libraries(check_encoders
        pendule_pi
        pendule_cli
)

# ----------------------------------------
add_executable(check_limit_switches
        src/bin/check_limit_switches.cpp
)
target_link_libraries(check_limit_switches
        pendule_pi
        pendule_cli
)

# ----------------------------------------
add_executable(check_motor
        src/bin/check_motor.cpp
)
target_link_libraries(check_motor
        pendule_pi
        pendule_cli
)
