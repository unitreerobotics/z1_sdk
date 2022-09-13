cmake_minimum_required(VERSION 3.0)
project(z1_sdk)

# set(EIGEN_PATH /usr/include/eigen3)

include_directories(
    include
    unitreeArm
    # ${EIGEN_PATH}
)

link_directories(lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread")

add_executable(example_lowCmd_send examples/example_lowCmd_send.cpp)
target_link_libraries(example_lowCmd_send Z1_SDK_Linux64)

add_executable(example_keyboard_send examples/example_keyboard_send.cpp)
target_link_libraries(example_keyboard_send Z1_SDK_Linux64)

add_executable(bigDemo examples/bigDemo.cpp)
target_link_libraries(bigDemo Z1_SDK_Linux64)

add_executable(getJointGripperState examples/getJointGripperState.cpp)
target_link_libraries(getJointGripperState Z1_SDK_Linux64)
