cmake_minimum_required(VERSION 2.8.3)
project(collision_detector_diagnoser)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        fusion_msgs
        pluginlib
        fault_core
        footprint_checker
        kinetic_energy_monitor
        dynamic_reconfigure
)

include_directories(ros/include
                    ${catkin_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  ros/cfg/diagnoser.cfg
)

catkin_package(
    INCLUDE_DIRS
      ros/include
    LIBRARIES collision_detector_diagnoser
    CATKIN_DEPENDS
        dynamic_reconfigure
        roscpp
        fusion_msgs
        pluginlib
        fault_core
        footprint_checker
        kinetic_energy_monitor
)

add_library(collision_detector_diagnoser ros/src/collision_detector_diagnoser.cpp)

add_executable(collision_detector_diagnoser_node
  ros/src/collision_detector_diagnoser_node.cpp
)

target_link_libraries(collision_detector_diagnoser ${catkin_LIBRARIES})
target_link_libraries(collision_detector_diagnoser_node collision_detector_diagnoser)

add_dependencies(collision_detector_diagnoser_node collision_detector_diagnoser)
add_dependencies(collision_detector_diagnoser_node ${PROJECT_NAME}_gencfg)

### INSTALLS
install(TARGETS
  collision_detector_diagnoser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
  collision_detector_diagnoser_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES fault_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
