cmake_minimum_required(VERSION 3.0.2)
project(add_box_pkg)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare the executables
add_executable(add_box_node src/add_box.cpp)
add_executable(move_robot src/move_robot.cpp)
add_executable(move_robot_2 src/move_robot_2.cpp)

## Link the executables to catkin libraries
target_link_libraries(add_box_node
  ${catkin_LIBRARIES}
)



target_link_libraries(move_robot_2
  ${catkin_LIBRARIES}
)

target_link_libraries(move_robot
  ${catkin_LIBRARIES}
)
