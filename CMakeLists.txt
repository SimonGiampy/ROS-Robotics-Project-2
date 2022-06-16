cmake_minimum_required(VERSION 3.0.2)
project(project2)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
)

catkin_package(

)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf2
  src/odometry.cpp
)
add_dependencies(odom_tf2 ${${project2}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_tf2
  ${catkin_LIBRARIES}
)
