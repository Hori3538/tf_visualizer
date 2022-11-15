cmake_minimum_required(VERSION 3.0.2)
project(tf_visualizer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    tf2_msgs
    )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_visualizer
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_executable(tf_visualizer_node src/tf_visualizer_node.cpp src/tf_visualizer.cpp)
add_dependencies(tf_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    tf_visualizer_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    )

