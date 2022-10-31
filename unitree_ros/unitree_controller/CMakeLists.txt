cmake_minimum_required(VERSION 2.8.3)
project(unitree_controller)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    sensor_msgs
    std_msgs
    unitree_legged_msgs
)

set(ENV{EIGEN3_INCLUDE_DIR} "/usr/include/eigen3")

find_package(gazebo REQUIRED)

catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
    LIBRARIES unitree_controller
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    $ENV{EIGEN3_INCLUDE_DIR}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare a C++ library
add_library(${PROJECT_NAME}
    src/body.cpp
    src/Transformations.cpp
    src/InverseKinematics.cpp
    src/RobotController/RobotController.cpp
    src/RobotController/RestController.cpp
    src/RobotController/TrotGaitController.cpp
    src/RobotController/GaitController.cpp
    src/RobotController/CrawlGaitController.cpp
    src/RobotController/PIDController.cpp
    src/RobotController/StandController.cpp
)

add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS}
)

# add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
# target_link_libraries(unitreeFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
# target_link_libraries(unitreeDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(unitree_external_force src/external_force.cpp)
target_link_libraries(unitree_external_force ${catkin_LIBRARIES})

add_executable(unitree_servo src/servo.cpp)
target_link_libraries(unitree_servo ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(unitree_control_servo src/servo_controller.cpp)
target_link_libraries(unitree_control_servo ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(unitree_example_tommy src/move_example_Tommy.cpp)
target_link_libraries(unitree_example_tommy ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(unitree_move_kinetic src/move_publisher.cpp)
target_link_libraries(unitree_move_kinetic ${catkin_LIBRARIES})

add_executable(unitree_move_tommy src/ex_Tommy.cpp)
target_link_libraries(unitree_move_tommy ${catkin_LIBRARIES})

add_executable(robot_controller_gazebo 
    src/main.cpp 
    src/InverseKinematics.cpp 
    src/Transformations.cpp
    src/RobotController/RobotController.cpp
    src/RobotController/RestController.cpp
    src/RobotController/TrotGaitController.cpp
    src/RobotController/GaitController.cpp
    src/RobotController/CrawlGaitController.cpp
    src/RobotController/PIDController.cpp
    src/RobotController/StandController.cpp
    )

#target_link_libraries(robot_controller_gazebo $ENV{EIGEN3_INCLUDE_DIR} ${catkin_LIBRARIES})
target_link_libraries(robot_controller_gazebo ${PROJECT_NAME} ${catkin_LIBRARIES})

# add_executable is actie die naam "unitree_move_kinetic" krijgt, die volgt van het script "move_publisher.cpp".
# target_link_libraries zorgt dat executable gebruik kan maken van bepaalde library, in dit geval, de libraries "${PROJECT_NAME}" in regel 35 en "${catkin_LIBRARIES}" van regel 4.

# wiki.ros.org/catkin/CMakeListst.txt --> more info.
