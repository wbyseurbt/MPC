cmake_minimum_required(VERSION 3.10)
project(legged_dm_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dm_hw
  sensor_msgs
  tf
  serial
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  dm_hw
  sensor_msgs
  serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(legged_dm_hw
  src/legged_dm_hw.cpp
  src/DmHW.cpp
)

target_link_libraries(legged_dm_hw
  ${catkin_LIBRARIES}
)

install(TARGETS legged_dm_hw
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
