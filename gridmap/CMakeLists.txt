cmake_minimum_required(VERSION 3.0.2)
project(gridmap)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  tf
)


find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gridmap
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${PYTHON_INCLUDE_DIRS})

add_executable(map src/map.cpp)
add_dependencies(map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(lidar src/lidarOccGrid.cpp)
add_dependencies(lidar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})

add_executable(fusion src/fusion.cpp)
add_dependencies(fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fusion ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})


