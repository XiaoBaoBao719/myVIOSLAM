cmake_minimum_required(VERSION 3.0)
project(mySLAM)

set( CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-std=c++14")

include_directories("usr/local/include/eigen3")

find_package(Pangolin)
find_package(OpenCV REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS} )

# add_library()
# add_executable(eigenMatrix src/eigenMatrix.cpp)
# add_executable(useGeometry src/useGeometry.cpp)
# add_executable(coordinateTransform src/coordinateTransform.cpp)
# add_executable(plotTrajectory src/plotTrajectory.cpp)
# add_executable(orbFeatureExtractor src/orb_cv.cpp)
add_executable(PoseEstimator2d2d src/pose_estimation_2d_2d.cpp)

# target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
# target_link_libraries(orbFeatureExtractor ${OpenCV_LIBRARIES})
target_link_libraries(PoseEstimator2d2d ${OpenCV_LIBRARIES})