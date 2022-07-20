cmake_minimum_required(VERSION 3.0)
project(z1_sdk)

# set(EIGEN_PATH /usr/include/eigen3)

include_directories(
    include
    # ${EIGEN_PATH}
)

link_directories(lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread")

# file(GLOB_RECURSE ADD_SRC_LIST
#     src/*.cpp
# )

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)                  # SHARED:CMAKE_LIBRARY_OUTPUT_DIRECTORY   STATIC:CMAKE_ARCHIVE_OUTPUT_DIRECTORY
# add_library(Z1_SDK_Linux64 SHARED ${ADD_SRC_LIST})

add_executable(example_lowCmd_send examples/example_lowCmd_send.cpp)
target_link_libraries(example_lowCmd_send Z1_SDK_Linux64)

add_executable(example_state_send examples/example_state_send.cpp)
target_link_libraries(example_state_send Z1_SDK_Linux64)

add_executable(example_state_recv examples/example_state_recv.cpp)
target_link_libraries(example_state_recv Z1_SDK_Linux64)

add_executable(example_keyboard_recv examples/example_keyboard_recv.cpp)
target_link_libraries(example_keyboard_recv Z1_SDK_Linux64)

add_executable(example_keyboard_send examples/example_keyboard_send.cpp)
target_link_libraries(example_keyboard_send Z1_SDK_Linux64)