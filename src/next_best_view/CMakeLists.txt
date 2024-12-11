# cmake_minimum_required(VERSION 3.22)
# project(next_best_view)

# # Common cmake code applied to all moveit packages
# find_package(moveit_common REQUIRED)
# moveit_package()

# # set(THIS_PACKAGE_INCLUDE_DIRS
# #   include
# # )

# set(THIS_PACKAGE_INCLUDE_DEPENDS
#   ament_cmake
#   Boost
#   control_msgs
#   Eigen3
#   interactive_markers
#   moveit_core
#   moveit_msgs
#   moveit_ros_planning
#   moveit_ros_planning_interface
#   moveit_servo
#   moveit_task_constructor_core
#   moveit_visual_tools
#   pluginlib
#   rclcpp
#   rclcpp_action
#   rviz_visual_tools
#   tf2_geometry_msgs
#   tf2_geometry_msgs
#   tf2_ros
# )

# foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()

# # include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

# add_executable(moveit_example src/hello_moveit.cpp)
# ament_target_dependencies(moveit_example ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ament_export_dependencies(
#   ${THIS_PACKAGE_INCLUDE_DEPENDS}
# )

# # ament_export_include_directories(include)

# ament_package()

cmake_minimum_required(VERSION 3.8)
project(next_best_view)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_cmake_python
  Boost
  control_msgs
  Eigen3
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

ament_python_install_package(next_best_view)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(next_best_view src/hello_moveit.cpp)
target_include_directories(next_best_view PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(next_best_view
  "rclcpp"
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )
target_compile_features(next_best_view PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS next_best_view
  DESTINATION lib/${PROJECT_NAME})

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
