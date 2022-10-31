cmake_minimum_required(VERSION 3.0.2)
project(a1_joystick)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  LIBRARIES a1_joystick
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ramped_joystick 
    src/main.cpp
    )
target_link_libraries(ramped_joystick ${catkin_LIBRARIES})


