cmake_minimum_required(VERSION 2.8.3)
project(chip_imu_driver)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  serial
  geometry_msgs
  sensor_msgs
  tf
  )

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs serial tf
  )

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include/chip_imu_driver ${catkin_INCLUDE_DIRS})

# Building
add_library(imu_serial src/imu_serial)
target_link_libraries(imu_serial ${catkin_LIBRARIES})

add_executable(chip_imu_driver_node src/chip_imu_driver_node.cpp)
target_link_libraries(chip_imu_driver_node imu_serial ${catkin_LIBRARIES})
set_target_properties(chip_imu_driver_node PROPERTIES OUTPUT_NAME chip_imu_driver_node PREFIX "")

install(TARGETS chip_imu_driver_node imu_serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# if more folders like config add below
#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
