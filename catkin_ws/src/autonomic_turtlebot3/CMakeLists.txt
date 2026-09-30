cmake_minimum_required(VERSION 3.0.2)
project(autonomic_turtlebot3)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf2
  tf2_geometry_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrt_global_planner
  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp tf2 tf2_geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(rrt_global_planner src/rrt_global_planner.cpp)
target_link_libraries(rrt_global_planner ${catkin_LIBRARIES})

install(TARGETS rrt_global_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/auxiliary_mqtt_bridge.py
  scripts/autonomic_manager.py
  scripts/battery_monitor.py
  scripts/knowledge_logger.py
  scripts/lift_controller.py
  scripts/mqtt_bridge.py
  scripts/safety_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch missions
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES rrt_global_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_policies.py)
  catkin_add_nosetests(test/test_secure_payload.py)
endif()
