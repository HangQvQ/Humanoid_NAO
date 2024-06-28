cmake_minimum_required(VERSION 3.0.2)
project(nao_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Components
find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  rospy
  std_msgs
  naoqi_bridge_msgs
  std_srvs
  tf
  actionlib_msgs
  geometry_msgs
  cv_bridge
  message_generation
  sensor_msgs
  aruco_ros
  image_transport
)

# Service Files
add_service_files(
  DIRECTORY srv
  FILES
  LookAround.srv
  TurnAround.srv
  BottleDetector.srv
  GetPos.srv
  NaoWalk.srv
  NaoWalkWithArms.srv
  detection.srv
  ArucoNavigation.srv
  ArucoDetector.srv
)

# Dependencies in SRV Files
generate_messages(DEPENDENCIES 
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Package
catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python
catkin_install_python(PROGRAMS src/get_head_angle.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/move_arms_rotate.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/move_arms_grasp.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/move_arms_down.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/move_arms_open.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/stand_pose.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/nao_walk.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/nao_walk_backwards.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/nao_walk_with_arms.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/detection_srv.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/walk_to_aruco.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# C++ Excecutables files
add_executable(aruco_detector_service src/aruco_detector.cpp)
target_link_libraries(aruco_detector_service ${catkin_LIBRARIES})

find_package(OpenCV REQUIRED)
target_link_libraries(aruco_detector_service ${OpenCV_LIBS})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake)
find_package(aruco REQUIRED)
target_link_libraries(aruco_detector_service  ${aruco_LIBS})

add_executable(look_around_service src/look_around.cpp)
target_link_libraries(look_around_service ${catkin_LIBRARIES})

add_executable(nao_turn_body_service src/nao_turn_body.cpp)
target_link_libraries(nao_turn_body_service ${catkin_LIBRARIES})

add_executable(bottle_detector_service src/bottle_detector.cpp)
target_link_libraries(bottle_detector_service ${catkin_LIBRARIES})

# Libraries
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(bottle_detector_service ${OpenCV_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(project src/main.cpp)
target_link_libraries(project ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})


