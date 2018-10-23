cmake_minimum_required(VERSION 2.8.3)
project(ilcc)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  camera_model
)

find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Ceres
find_package(Ceres REQUIRED)
include_directories( ${CERES_INCLUDE_DIRS} )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ILCC
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(pcd_corners src/pcd_corners.cpp
    src/extractCorner/cornersFromTXT.cpp
    src/extractCorner/Optimization.cpp
    src/extractCorner/Visualization.cpp)
target_link_libraries(pcd_corners ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable (pcd2image src/pcd2image.cpp)
target_link_libraries (pcd2image ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable (pcd_get_board src/pcd_get_board.cpp)
target_link_libraries (pcd_get_board ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable (camera_corners src/camera_corners.cpp)
target_link_libraries (camera_corners camera_model ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} )

add_executable (lidar_cam_calib
    src/lidar_cam_calib.cpp
    src/extractCorner/cornersFromTXT.cpp
    src/extractCorner/Optimization.cpp
    src/extractCorner/Visualization.cpp )

target_link_libraries (lidar_cam_calib
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES})


add_executable (calibration_validator src/calibration_validator.cpp)
target_link_libraries (calibration_validator ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(getBoardCorners src/getBoardCorners.cc)
target_link_libraries(getBoardCorners ${catkin_LIBRARIES} camera_model ${OpenCV_LIBS} )

