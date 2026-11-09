cmake_minimum_required(VERSION 3.19.4)
project(pendule_pi VERSION 0.3.1 LANGUAGES CXX)

# Build Release by default : the control loop must run with same optimizations
# as the demonstrator (see README)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Type de build" FORCE)
endif ()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies : fatal error if one is missing
# (zmq et zmqpp are linked by name, without using find_package)
find_package(pigpio REQUIRED)
find_package(yaml-cpp REQUIRED)

# Warnings on all targets. Optimization level comes from CMAKE_BUILD_TYPE
# (Release = -O3 -DNDEBUG).
add_compile_options(-Wall)

# code shared by demo and low_level_interface

add_library(pendule_pi
        src/pendule_pi/config.cpp
        src/pendule_pi/joystick.cpp
        src/pendule_pi/pigpio.cpp
        src/pendule_pi/switch.cpp
        src/pendule_pi/motor.cpp
        src/pendule_pi/encoder.cpp
        src/pendule_pi/pendule.cpp
        src/pendule_pi/pendule_cpp_server.cpp
)

# inject the version string as defined in the project() clause
target_compile_definitions(pendule_pi PUBLIC PENDULE_PI_VERSION="${PROJECT_VERSION}")

target_include_directories(pendule_pi PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(pendule_pi PUBLIC
        pigpio::pigpio
        pthread
        zmq
        zmqpp
        yaml-cpp
)

target_compile_definitions(pendule_pi PUBLIC
        $<$<CONFIG:Debug>:PENDULE_PI_DEBUG_ENABLED>
)

###
### Binaries
###

# ----------------------------------------
add_executable(low_level_interface
        src/bin/low_level_interface.cpp
)
target_link_libraries(
        low_level_interface
        pendule_pi
)

# ----------------------------------------
add_executable(demo
        src/bin/demo.cpp
        src/pendule_pi/pendule_cpp_client.cpp
)
target_link_libraries(demo
        pendule_pi
        zmq
        zmqpp
)

# ----------------------------------------
add_executable(reset_gpio
        src/bin/reset_gpio.cpp
)
target_link_libraries(reset_gpio
        pendule_pi
)
