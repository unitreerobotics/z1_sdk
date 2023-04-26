cmake_minimum_required(VERSION 3.0)
project(z1_sdk)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread")
set(CMAKE_CXX_STANDARD_REQUIRED on)
set(CMAKE_CXX_STANDARD 14)
set(EIGEN_PATH /usr/include/eigen3)
include_directories(
    include
    ${EIGEN_PATH}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
link_directories(lib)

add_executable(highcmd_basic examples/highcmd_basic.cpp )
target_link_libraries(highcmd_basic Z1_SDK_${CMAKE_HOST_SYSTEM_PROCESSOR})
add_executable(highcmd_development examples/highcmd_development.cpp )
target_link_libraries(highcmd_development Z1_SDK_${CMAKE_HOST_SYSTEM_PROCESSOR})
add_executable(lowcmd_development examples/lowcmd_development.cpp )
target_link_libraries(lowcmd_development Z1_SDK_${CMAKE_HOST_SYSTEM_PROCESSOR})
add_executable(lowcmd_multirobots examples/lowcmd_multirobots.cpp )
target_link_libraries(lowcmd_multirobots Z1_SDK_${CMAKE_HOST_SYSTEM_PROCESSOR})

add_subdirectory(pybind11)
pybind11_add_module(unitree_arm_interface examples_py/arm_python_interface.cpp)
target_link_libraries(unitree_arm_interface PRIVATE Z1_SDK_${CMAKE_HOST_SYSTEM_PROCESSOR})
