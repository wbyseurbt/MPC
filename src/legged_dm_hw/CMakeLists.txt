cmake_minimum_required(VERSION 3.10)
project(legged_dm_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

# Outer workspaces often prepend CMAKE_PREFIX_PATH (e.g. motor_ws also ships a `dm_hw`
# without IMU / legged_common). Catkin then picks that package first. Pin to this
# workspace's dm_hw when it has been built here (same layout on x86_64 and arm64).
get_filename_component(_MPC_WS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(_MPC_DM_HW_CONFIG "${_MPC_WS_ROOT}/devel/share/dm_hw/cmake/dm_hwConfig.cmake")
if(EXISTS "${_MPC_DM_HW_CONFIG}")
  set(dm_hw_DIR "${_MPC_WS_ROOT}/devel/share/dm_hw/cmake" CACHE PATH "" FORCE)
endif()

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
