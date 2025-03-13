cmake_minimum_required(VERSION 3.8)
project(next_best_view)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set CMake policy CMP0074 to NEW to suppress the warning
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kinova_action_interfaces REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(kinova_gen3_6dof_robotiq_2f_85_moveit_config REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(rcpputils REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(Eigen3 REQUIRED)
# Octomap
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(glog REQUIRED)  # Added for glog
find_package(OpenMP REQUIRED)  # Added for OpenMP
find_package(CGAL REQUIRED)  # Added for CGAL
find_package(rosidl_default_generators REQUIRED)  # Added for service generation

find_package(PCL REQUIRED COMPONENTS common io filters)

# Add executables
add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(moveto src/moveto.cpp)
add_executable(nextbestview src/nextbestview.cpp)
add_executable(point_cloud_saver src/point_cloud_saver.cpp)
add_executable(merge_point_clouds src/merge_point_clouds.cpp)
add_executable(canopy_volume src/canopy_volume.cpp)
add_executable(object_detection src/object_detection.cpp)
add_executable(octomap_generate src/octomap_generate.cpp)
add_executable(point_cloud_filter src/point_cloud_filter.cpp)
add_executable(clear_files src/clear_files.cpp)
add_executable(voxel_map src/voxel_map.cpp)
add_executable(nextbestview_compute src/nextbestview_compute.cpp)
add_executable(pose_generator src/pose_generator.cpp)  # New service node
add_executable(reachability_check src/reachability_check.cpp)  # New service node
add_executable(collision_object src/collision_object.cpp)  # New executable
add_executable(movetopose src/movetopose.cpp)  # New executable
add_executable(currentpose src/currentpose.cpp)  # New executable
add_executable(coordinator src/coordinator.cpp)  # New executable




# Set include directories for each target
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(moveto PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(nextbestview PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(point_cloud_saver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(merge_point_clouds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(canopy_volume PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(object_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(octomap_generate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(point_cloud_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(clear_files PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(voxel_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})

target_include_directories(nextbestview_compute PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS})

target_include_directories(pose_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  # Include dirs for new service node

target_include_directories(reachability_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  # Include dirs for new service node

target_include_directories(collision_object PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  # Include dirs for new collision_object executable

target_include_directories(movetopose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  # Include dirs for new collision_object executable

target_include_directories(currentpose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  # Include dirs for new collision_object executable

target_include_directories(coordinator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  # Include dirs for new collision_object executable

# Set compile features and dependencies for each target
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
)

target_compile_features(moveto PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  moveto
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "std_msgs"
  "kinova_action_interfaces"
)

target_compile_features(nextbestview PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  nextbestview
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "std_msgs"
  "kinova_action_interfaces"
  "moveit_visual_tools"
  "tf2"
  "rviz_visual_tools"
  "geometric_shapes"
  "octomap"
  "octomap_msgs"
  "pcl_conversions"
  "pcl_ros"
  "PCL"
  "sensor_msgs"
)

target_compile_features(point_cloud_saver PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  point_cloud_saver
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "pcl_ros"
  "PCL"
  "tf2_sensor_msgs"
  "tf2"
  "tf2_ros"
  "rcpputils"
  "kinova_action_interfaces"
)

target_compile_features(merge_point_clouds PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  merge_point_clouds
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "pcl_ros"
  "PCL"
  "message_filters"
  "cv_bridge"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "OpenCV"
  "tf2_sensor_msgs"
  "kinova_action_interfaces"
)

target_compile_features(canopy_volume PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  canopy_volume
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "pcl_ros"
  "PCL"
  "message_filters"
  "cv_bridge"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "OpenCV"
  "tf2_sensor_msgs"
  "kinova_action_interfaces"
)

target_compile_features(object_detection PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  object_detection
  "rclcpp"
  "rclcpp_action"
  "message_filters"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "message_filters"
  "cv_bridge"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "OpenCV"
  "Eigen3"
  "kinova_action_interfaces"
)
include_directories(${EIGEN3_INCLUDE_DIR})

target_compile_features(octomap_generate PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  octomap_generate
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "pcl_ros"
  "PCL"
  "message_filters"
  "tf2"
  "tf2_ros"
  "tf2_eigen"
  "geometry_msgs"
  "tf2_sensor_msgs"
  "octomap"
  "octomap_msgs"
)

target_compile_features(point_cloud_filter PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  point_cloud_filter
  "rclcpp"
  "rclcpp_action"
  "std_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "pcl_ros"
  "PCL"
  "message_filters"
  "kinova_action_interfaces"
)

target_compile_features(clear_files PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  clear_files
  "rclcpp"
  "rclcpp_action"
  "kinova_action_interfaces"
)

target_compile_features(voxel_map PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  voxel_map
  "rclcpp"
  "rclcpp_action"
  "sensor_msgs"
  "geometry_msgs"
  "octomap_msgs"
  "pcl_conversions"
  "tf2_eigen"
  "kinova_action_interfaces"
)

target_compile_features(nextbestview_compute PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  nextbestview_compute
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "octomap_msgs"
  "tf2_eigen"
  "kinova_action_interfaces"
  "OpenCV"
  "CGAL"
)

target_compile_features(pose_generator PUBLIC c_std_99 cxx_std_17)  # New for service node
ament_target_dependencies(
  pose_generator
  "rclcpp"
  "geometry_msgs"
  "kinova_action_interfaces"
)

target_compile_features(reachability_check PUBLIC c_std_99 cxx_std_17)  # New for service node
ament_target_dependencies(
  reachability_check
  "rclcpp"
  "kinova_action_interfaces"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "moveit_ros_planning_interface"
)

target_compile_features(collision_object PUBLIC c_std_99 cxx_std_17)  # New for collision_object
ament_target_dependencies(
  collision_object
  "rclcpp"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "moveit_msgs"
  "shape_msgs"
  "kinova_action_interfaces"

)

target_compile_features(movetopose PUBLIC c_std_99 cxx_std_17)  # New for collision_object
ament_target_dependencies(
  movetopose
  "rclcpp"
  "rclcpp_action"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "moveit_msgs"
  "kinova_action_interfaces"

)

target_compile_features(currentpose PUBLIC c_std_99 cxx_std_17)  # New for collision_object
ament_target_dependencies(
  currentpose
  "rclcpp"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "moveit_msgs"
  "kinova_action_interfaces"
  "tf2_ros"
  "tf2_geometry_msgs"
)

target_compile_features(coordinator PUBLIC c_std_99 cxx_std_17)  # New for collision_object
ament_target_dependencies(
  coordinator
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "kinova_action_interfaces"

)

# Link libraries for specific targets
target_link_libraries(voxel_map
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  glog::glog
  OpenMP::OpenMP_CXX)

target_link_libraries(nextbestview_compute
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  glog::glog
  OpenMP::OpenMP_CXX
  ${OpenCV_LIBRARIES}
  ${CGAL_LIBRARIES})

# Install targets
install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS moveto
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS nextbestview
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS point_cloud_saver
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS merge_point_clouds
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS canopy_volume
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS object_detection
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS octomap_generate
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS point_cloud_filter
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS clear_files
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS voxel_map
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS nextbestview_compute
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pose_generator
  DESTINATION lib/${PROJECT_NAME})  

install(TARGETS reachability_check
  DESTINATION lib/${PROJECT_NAME})   

install(TARGETS collision_object
  DESTINATION lib/${PROJECT_NAME})  

install(TARGETS movetopose
  DESTINATION lib/${PROJECT_NAME})  

install(TARGETS currentpose
  DESTINATION lib/${PROJECT_NAME})  

install(TARGETS coordinator
  DESTINATION lib/${PROJECT_NAME})  

  # Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/object_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
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