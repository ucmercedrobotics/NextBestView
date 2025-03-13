cmake_minimum_required(VERSION 3.8)
project(kinova_action_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  sensor_msgs
  geometry_msgs
  std_msgs
  octomap_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DetectObject.action"
  "action/MoveTo.action"
  "action/NextBestView.action"
  "action/FilterPointCloud.action"
  "action/SavePointCloud.action"
  "action/MergePointClouds.action"
  "action/ClearFiles.action"
  "action/ProcessTreeData.action"
  "action/VoxelMap.action"
  "action/ComputeNextbestview.action"
  "srv/PoseGenerator.srv"
  "srv/ReachabilityCheck.srv"
  "srv/AddCollisionObject.srv"
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs octomap_msgs
)



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
