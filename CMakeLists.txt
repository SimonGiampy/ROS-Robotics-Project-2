cmake_minimum_required(VERSION 3.0.2)
project(project2)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf2_ros
    std_msgs
    message_generation
)

add_service_files(FILES TrajectorySrv.srv)

generate_messages(
    DEPENDENCIES
    std_msgs
)


catkin_package(

)

catkin_install_python(PROGRAMS scripts/trajectory_map.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf2
    src/odom_tf2.cpp
)

add_dependencies(odom_tf2 ${${project2}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(odom_tf2
    ${catkin_LIBRARIES}
)
