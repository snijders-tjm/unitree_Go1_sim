cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_real)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    unitree_legged_msgs
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    std_msgs
    tf
    sensor_msgs
)

set(ENV{EIGEN3_INCLUDE_DIR} "/usr/include/eigen3")

catkin_package()

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/${ARCH})

set(EXTRA_LIBS -pthread libunitree_legged_sdk.so)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
    $ENV{EIGEN3_INCLUDE_DIR}
)






add_executable(example_walk src/exe/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(example_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(example_position src/exe/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(example_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(state_sub src/exe/state_sub.cpp)
target_link_libraries(state_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(state_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_udp src/exe/ros_udp.cpp)
target_link_libraries(ros_udp ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_udp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(control_via_keyboard src/exe/control_via_keyboard.cpp)
target_link_libraries(control_via_keyboard ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(control_via_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(twist_sub src/exe/twist_sub.cpp)
target_link_libraries(twist_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(twist_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(LL_walk 
    src/exe/code_LL_walk.cpp
    src/exe/body.cpp
    src/exe/InverseKinematics.cpp 
    src/exe/Transformations.cpp
    src/exe/RobotController/RobotController.cpp
    src/exe/RobotController/RestController.cpp
    src/exe/RobotController/TrotGaitController.cpp
    src/exe/RobotController/GaitController.cpp
    src/exe/RobotController/CrawlGaitController.cpp
    src/exe/RobotController/PIDController.cpp
    src/exe/RobotController/StandController.cpp
    )
target_link_libraries(LL_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(LL_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

