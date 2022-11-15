cmake_minimum_required(VERSION 3.0)
project(z1_sdk)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread")

set(EIGEN_PATH /usr/include/eigen3)
include_directories(
    include/unitree_arm_sdk
    ${EIGEN_PATH}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)

link_directories(lib)

add_executable(bigDemo examples/bigDemo.cpp )
target_link_libraries(bigDemo Z1_SDK_Linux64)
add_executable(example_lowCmd_send examples/example_lowCmd_send.cpp )
target_link_libraries(example_lowCmd_send Z1_SDK_Linux64)
add_executable(example_CtrlByFSM examples/example_CtrlByFSM.cpp )
target_link_libraries(example_CtrlByFSM Z1_SDK_Linux64)
add_executable(example_JointCtrl examples/example_JointCtrl.cpp )
target_link_libraries(example_JointCtrl Z1_SDK_Linux64)