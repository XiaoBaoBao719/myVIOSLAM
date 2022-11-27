cmake_minimum_required(VERSION 3.0)
project(mySLAM)

set( CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-std=c++14")

include_directories("usr/local/include/eigen3")

find_package(Pangolin)
include_directories(${Pangolin_INCLUDE_DIRS} )

# add_library()
add_executable(eigenMatrix src/eigenMatrix.cpp)
add_executable(useGeometry src/useGeometry.cpp)
add_executable(coordinateTransform src/coordinateTransform.cpp)
add_executable(plotTrajectory src/plotTrajectory.cpp)

target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})