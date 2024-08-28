cmake_minimum_required(VERSION 3.8)
project(erl_quadrotor_mocap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_capture_tracking_interfaces REQUIRED)
find_package(mocap_optitrack_interfaces REQUIRED)
ament_export_dependencies(
  Eigen3
  tf2_ros
  nav_msgs
  geometry_msgs
)

ament_export_include_directories(${Eigen_INCLUDE_DIRS})
add_executable(erl_quadrotor_mocap_node src/erl_quadrotor_mocap.cpp src/filter.cpp)
ament_target_dependencies(
  erl_quadrotor_mocap_node
  tf2_ros
  nav_msgs
  geometry_msgs
  motion_capture_tracking_interfaces
  mocap_optitrack_interfaces
)

target_include_directories(erl_quadrotor_mocap_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(erl_quadrotor_mocap_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS erl_quadrotor_mocap_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
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
