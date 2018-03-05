cmake_minimum_required(VERSION 2.8.3)
project(static_obstacles_recovery)

find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        roscpp
        pluginlib
        fault_core
        geometry_msgs
        std_srvs
        )

include_directories(ros/include
                    ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS
      ros/include
    LIBRARIES static_obstacles_recovery
    CATKIN_DEPENDS
        dynamic_reconfigure
        roscpp
        pluginlib
        fault_core
        geometry_msgs
        std_srvs
)

add_library(mislocalization_collision_recovery ros/src/static_obstacles_recovery.cpp)

add_executable(static_obstacles_recovery_node
  ros/script/static_obstacles_recovery.cpp
)

target_link_libraries(static_obstacles_recovery ${catkin_LIBRARIES})
target_link_libraries(static_obstacles_recovery_node static_obstacles_recovery)

add_dependencies(static_obstacles_recovery_node static_obstacles_recovery)

### INSTALLS
install(TARGETS static_obstacles_recovery
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
  static_obstacles_recovery_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES fault_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)
