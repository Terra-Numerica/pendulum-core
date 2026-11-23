cmake_minimum_required(VERSION 3.19.4)
project(pendulum_core VERSION 0.6.0 LANGUAGES CXX)

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

# Header-only libraries living in include/
add_library(yamlcfg INTERFACE)
target_include_directories(yamlcfg INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(yamlcfg INTERFACE
        yaml-cpp::yaml-cpp
)

add_library(digital_filters INTERFACE)
target_include_directories(digital_filters INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Code shared by the executables
add_library(pendulum_core
        src/pendulum_core/joystick.cpp
        src/pendulum_core/pigpio.cpp
        src/pendulum_core/switch.cpp
        src/pendulum_core/motor.cpp
        src/pendulum_core/encoder.cpp
        src/pendulum_core/pendulum.cpp
        src/pendulum_core/pendulum_cpp_server.cpp
        src/pendulum_core/pendulum_cpp_client.cpp
        src/pendulum_core/pendulum_config.cpp
        src/pendulum_core/sockets_config.cpp
)

target_include_directories(pendulum_core PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# PUBLIC : every binary linked to pendulum_core inherits these dependencies
target_link_libraries(pendulum_core PUBLIC
        pigpio::pigpio
        Threads::Threads
        zmq
        zmqpp
        yamlcfg
)

target_compile_definitions(pendulum_core PUBLIC
        PENDULUM_CORE_VERSION="${PROJECT_VERSION}"
        $<$<CONFIG:Debug>:PENDULUM_CORE_DEBUG_ENABLED>
)

# Command line parsing shared by the executables. Built once as a library : CLI11 is expensive to compile
# (header-only), so it is isolated in cli_parser.cpp and precompiled.
add_library(pendulum_cli STATIC
        src/pendulum_cli/cli_parser.cpp
        include/pendulum_cli/cli_parser.hpp
)
target_include_directories(pendulum_cli PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(pendulum_cli PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)
target_compile_definitions(pendulum_cli PRIVATE
        PENDULUM_CORE_VERSION="${PROJECT_VERSION}"
)
target_precompile_headers(pendulum_cli PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/CLI11/CLI11.hpp
)

###
### Binaries
###

# ----------------------------------------
add_executable(hw_interface
        src/bin/hw_interface/hw_interface.cpp
        src/bin/common/utils.hpp
        include/pendulum_core/pendulum_config.hpp
        include/pendulum_core/sockets_config.hpp
)
target_link_libraries(hw_interface PRIVATE
        pendulum_core
        pendulum_cli
        digital_filters
)

# ----------------------------------------
add_executable(demo
        src/bin/demo/demo.cpp
        src/bin/demo/demo_config.cpp
        src/bin/demo/demo_config.hpp
        src/bin/common/utils.hpp
        include/pendulum_core/sockets_config.hpp
)
target_link_libraries(demo PRIVATE
        pendulum_core
        pendulum_cli
)

# ----------------------------------------
add_executable(reset_gpio
        src/bin/tools/reset_gpio.cpp
)
target_link_libraries(reset_gpio
        pendulum_core
        pendulum_cli
)

# ----------------------------------------
add_executable(check_encoders
        src/bin/tools/check_encoders.cpp
)
target_link_libraries(check_encoders
        pendulum_core
        pendulum_cli
)

# ----------------------------------------
add_executable(check_limit_switches
        src/bin/tools/check_limit_switches.cpp
)
target_link_libraries(check_limit_switches
        pendulum_core
        pendulum_cli
)

# ----------------------------------------
add_executable(check_motor
        src/bin/tools/check_motor.cpp
)
target_link_libraries(check_motor
        pendulum_core
        pendulum_cli
)
