cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 11)

project(gtsam_tutorial)
find_package(GTSAM REQUIRED)
find_package(Boost 1.65)

set(EIGEN3_DIR /usr/include/eigen3)
include_directories(${EIGEN3_DIR})
include_directories(${GTSAM_INCLUDE_DIR})

link_directories(${GTSAM_LIBRARY_DIRS} )

add_executable(tutorial ../main.cpp)
get_target_property(gtsam_link_lib gtsam INTERFACE_LINK_LIBRARIES)
target_link_libraries(tutorial PRIVATE gtsam)
