cmake_minimum_required(VERSION 3.0.2)
project(human_tracking_particle_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  dynamic_reconfigure
  sensor_msgs
  pedsim_msgs
  geometry_msgs
  costmap_2d
  # Grid Map Features
  grid_map_ros
  # Interactive Marker Features
  visualization_msgs
  interactive_markers
  # HTPF Features
  # human_tracking_particle_filter
)

# Generate dynamic reconfigure parameters in the 'cfg' folder
# Must be called before catkin_package()
generate_dynamic_reconfigure_options(
  cfg/HTPFLayer.cfg
  cfg/particleFilter.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES human_tracking_particle_filter
#  CATKIN_DEPENDS costmap_2d dynamic_reconfigure grid_map_ros interactive_markers roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Create human tracking particle filter core
add_library(${PROJECT_NAME}_core
  src/human_tracking_particle_filter.cpp
  src/grid_map_interface.cpp
)
add_dependencies(${PROJECT_NAME}_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES})

# Create human tracking particle filter core
add_library(${PROJECT_NAME} src/costmap_plugin.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME}_core)

# Create node
add_executable(${PROJECT_NAME}_node
 src/human_tracking_particle_filter_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

# Install node
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install costmap_plugins.xml
install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install library
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install include directories
install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
