cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME project)
project(${PROJECT_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  trajectory_msgs
  Eigen3
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(controller_component SHARED src/controller.cpp)
target_include_directories(controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(controller_component ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_node(controller_component PLUGIN "project::Controller" EXECUTABLE controller)

add_library(phyphox_streamer_component SHARED src/phyphox_streamer.cpp)
target_include_directories(phyphox_streamer_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(phyphox_streamer_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(phyphox_streamer_component ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_node(phyphox_streamer_component PLUGIN "project::PhyhoxStreamer" EXECUTABLE phyphox_streamer)

add_executable(controller_main src/controller_main.cpp)
target_include_directories(controller_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(controller_main controller_component)
ament_target_dependencies(controller_component ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_phyphox_streamer_component)
install(TARGETS phyphox_streamer_component
        EXPORT export_phyphox_streamer_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
ament_export_targets(export_controller_component)
install(TARGETS controller_component
        EXPORT export_controller_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(TARGETS controller_main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
