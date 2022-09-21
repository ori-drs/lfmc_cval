cmake_minimum_required(VERSION 3.10)
project(lfmc)

set(CMAKE_CXX_STANDARD 14)

# Eigen3
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

# YAML-CPP
find_package(yaml-cpp REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} include dependencies/networks_minimal/include)

# Make the whole library
file(GLOB SOURCES src/*.cpp)
file(GLOB HEADERS include/*.hpp)
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} PUBLIC ${YAML_CPP_LIBRARIES})

### RaiSim Executable
list(APPEND CMAKE_PREFIX_PATH $ENV{LOCAL_INSTALL})

find_package(raisim CONFIG REQUIRED)

add_executable(command_tracking examples/command_tracking.cpp)
target_link_libraries(command_tracking PUBLIC ${PROJECT_NAME} stdc++fs raisim::raisim pthread)
