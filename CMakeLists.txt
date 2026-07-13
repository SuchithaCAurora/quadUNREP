cmake_minimum_required(VERSION 3.14.4)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
project(trajectory_generator_ros2)

# add_compile_options("-O3")

# check c++14 / c++0x
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
# if(COMPILER_SUPPORTS_CXX14)
#     set(CMAKE_CXX_FLAGS "-std=c++14")
# else()
#     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support.")
# endif()

# check c++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-std=c++11 -Wall -Werror)
 
find_package(ament_cmake)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(snapstack_msgs2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
###########
## Build ##
###########

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/trajectory_generator_node.cpp
)

add_executable(follower_traj_generator
  src/follower_trajectory_node.cpp
)

add_executable(follower_traj_generator2
  src/follower_trajecotry_node2.cpp
)

add_executable(goal_visualizer
  src/goal_visualizer.cpp
)
################## Replacing catkin_package() ####################

## Declare a cpp library
add_library(${PROJECT_NAME}_core
  src/TrajectoryGenerator.cpp
  src/trajectories/Circle.cpp
  src/trajectories/Line.cpp
  src/trajectories/Figure8.cpp
  src/trajectories/Square.cpp
  src/trajectories/Reciprocating.cpp
  src/trajectories/Rectangle.cpp
  src/trajectories/Bounce.cpp
  src/trajectories/M.cpp
  src/trajectories/I.cpp
  src/trajectories/T.cpp
)

# ROS2:
# Replaces INCLUDE_DIRS keyword of catkin_package() call
target_include_directories(${PROJECT_NAME} PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${EIGEN3_INCLUDE_DIR})

target_include_directories(follower_traj_generator PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${EIGEN3_INCLUDE_DIR})

target_include_directories(follower_traj_generator2 PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${EIGEN3_INCLUDE_DIR})

target_include_directories(goal_visualizer PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${EIGEN3_INCLUDE_DIR})
## Specify libraries to link executable targets against
#target_link_libraries(${PROJECT_NAME}_core
#  ${catkin_LIBRARIES}
#)

# target_link_libraries(${PROJECT_NAME} PUBLIC
#   ${PROJECT_NAME}_core
#   ${ament_TARGETS}
#   rclcpp::rclcpp
#   # snapstack_msgs2::snapstack_msgs2 
#   ${geometry_msgs_TARGETS} 
#   ${std_msgs_TARGETS}
# )

#############
## Install ##
#############

include_directories(include
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  snapstack_msgs2
  std_msgs
  geometry_msgs
  Eigen3
)

ament_target_dependencies(${PROJECT_NAME}_core 
  rclcpp
  std_msgs
  geometry_msgs
  snapstack_msgs2
  Eigen3 
)

ament_target_dependencies(follower_traj_generator
  rclcpp
  std_msgs
  geometry_msgs
  snapstack_msgs2
  Eigen3 
)

ament_target_dependencies(follower_traj_generator2
  rclcpp
  std_msgs
  geometry_msgs
  snapstack_msgs2
  Eigen3 
)

ament_target_dependencies(goal_visualizer
  rclcpp
  snapstack_msgs2
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

# ROS2:
# Replaces LIBRARIES keyword from catkin_package()
# Mark executables and/or libraries for installation 
# install(TARGETS ${PROJECT_NAME} 
#   follower_traj_generator
#   EXPORT export_package
#   ARCHIVE DESTINATION lib 
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME} 
# )
install(TARGETS ${PROJECT_NAME} 
  follower_traj_generator2
  goal_visualizer
  EXPORT export_package
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} 
)

# Mark header files for installation 
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.hpp"
)

install (
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install (
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export for downstream users
ament_export_targets(export_package) 

# ROS2: 
# Replaces CATKIN_DEPENDS keyword from catkin_package()
ament_export_dependencies(
  rclcpp 
  snapstack_msgs2 
  geometry_msgs 
  std_msgs 
)

ament_package()