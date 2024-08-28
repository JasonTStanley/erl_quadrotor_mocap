cmake_minimum_required(VERSION 2.8.3)
project(erl_quadrotor_mocap)

find_package(catkin REQUIRED COMPONENTS tf2_ros cmake_modules mav_msgs)
find_package(Eigen REQUIRED)

catkin_package(CATKIN_DEPENDS tf2_ros
  DEPENDS Eigen)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/erl_quadrotor_mocap.cpp src/filter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
