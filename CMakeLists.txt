cmake_minimum_required(VERSION 3.16)
project(rpi_epic_canbus VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags for optimization
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# Default to Release build
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Find libgpiod
find_library(GPIOD_LIBRARY gpiod REQUIRED)
find_path(GPIOD_INCLUDE_DIR gpiod.h REQUIRED)

# Source files
set(SOURCES
    src/main.cpp
    src/can_interface.cpp
    src/gps_interface.cpp
    src/adc_interface.cpp
    src/gpio_interface.cpp
    src/nmea_parser.cpp
    src/config.cpp
)

# Header files
set(HEADERS
    include/epic_protocol.h
    include/can_interface.h
    include/gps_interface.h
    include/adc_interface.h
    include/gpio_interface.h
    include/nmea_parser.h
    include/config.h
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GPIOD_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${GPIOD_LIBRARY}
    pthread
)

# Install target
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# Install systemd service file
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/rpi_epic_canbus.service
    DESTINATION /etc/systemd/system
    COMPONENT config
)

# Print build info
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "libgpiod: ${GPIOD_LIBRARY}")
