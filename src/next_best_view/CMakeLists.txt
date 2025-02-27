cmake_minimum_required(VERSION 3.8)
project(next_best_view)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CMake Warning (dev) at CMakeLists.txt:31 (find_package):
#   Policy CMP0074 is not set: find_package uses <PackageName>_ROOT variables.
#   Run "cmake --help-policy CMP0074" for policy details.  Use the cmake_policy
#   command to set the policy and suppress this warning.

#   CMake variable PCL_ROOT is set to:

#     /usr

#   For compatibility, CMake is ignoring the variable.
# This warning is for project developers.  Use -Wno-dev to suppress it.


# Set CMake policy CMP0074 to NEW to suppress the warning
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()


# find dependencies
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

find_package(PCL REQUIRED COMPONENTS common io filters)

add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(moveto src/moveto.cpp)
add_executable(nextbestview src/nextbestview.cpp)
add_executable(point_cloud_saver src/point_cloud_saver.cpp)
add_executable(create_point_cloud2 src/create_point_cloud2.cpp)
add_executable(merge_point_clouds src/merge_point_clouds.cpp)
add_executable(canopy_volume src/canopy_volume.cpp)
add_executable(point_cloud_publisher src/point_cloud_publisher.cpp)
add_executable(object_detection src/object_detection.cpp)
add_executable(octomap_generate src/octomap_generate.cpp)
add_executable(point_cloud_filter src/point_cloud_filter.cpp)




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

target_include_directories(create_point_cloud2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(merge_point_clouds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(canopy_volume PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(point_cloud_publisher PUBLIC
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

target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
)

target_compile_features(moveto PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveto
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "std_msgs"
  "kinova_action_interfaces"

)

target_compile_features(nextbestview PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
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

target_compile_features(point_cloud_saver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  point_cloud_saver
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "pcl_ros"
  "PCL"
  "tf2_sensor_msgs"
  "tf2"
  "tf2_ros"
  "rcpputils"
)

target_compile_features(create_point_cloud2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  create_point_cloud2
  "rclcpp"
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

)

target_compile_features(merge_point_clouds PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  merge_point_clouds
  "rclcpp"
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

)

target_compile_features(canopy_volume PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  canopy_volume
  "rclcpp"
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

)

target_compile_features(point_cloud_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  point_cloud_publisher
  "rclcpp"
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

)

target_compile_features(object_detection PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
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
  "tf2_ros"
  "tf2_geometry_msgs"
  "OpenCV"
  "Eigen3"
  "kinova_action_interfaces"
)
include_directories(${EIGEN3_INCLUDE_DIR})

target_compile_features(octomap_generate PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
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

target_compile_features(point_cloud_filter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  point_cloud_filter
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "pcl_ros"
  "PCL"
  "message_filters"
  "kinova_action_interfaces"
)

install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS moveto
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS nextbestview
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS point_cloud_saver
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS create_point_cloud2
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS merge_point_clouds
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS canopy_volume
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS point_cloud_publisher
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS object_detection
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS octomap_generate
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS point_cloud_filter
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables in the ros2_ws/install/cpp_py_pkg/lib/cpp_py_pkg/ folder
install(PROGRAMS
scripts/object_detection.py
DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
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
