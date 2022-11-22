cmake_minimum_required(VERSION 3.16)
project(pcl_processor VERSION 0.0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pluginlib)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  pcl_conversions
  pluginlib
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  visualization_msgs
)

add_executable(processor_xyz
  src/processor_xyz.cpp
)

target_include_directories(processor_xyz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(processor_xyz ${dependencies})

target_link_libraries(processor_xyz ${PCL_LIBRARIES})

install(
  TARGETS processor_xyz
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Pointcloud Processor Plugins
add_library(pcl_processor_plugins SHARED
  # Filters
  src/filters/voxel_grid.cpp
  src/filters/crop_box.cpp
  src/filters/radius_outlier_removal.cpp
  # Segmentation
  src/segmentation/extract_clusters.cpp
  src/segmentation/ground_plane_fitting.cpp)

target_include_directories(pcl_processor_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pcl_processor_plugins ${dependencies})
target_link_libraries(pcl_processor_plugins ${PCL_LIBRARIES})

install(
  TARGETS pcl_processor_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(pcl_processor plugins.xml)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Disable cpplint and manually call it with filters later
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_xmllint ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(FILTERS "-legal/copyright" "-build/header_guard" "-build/include_order")
endif()

ament_package()
