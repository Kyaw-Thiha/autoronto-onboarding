cmake_minimum_required(VERSION 3.8)
project(sum_solver_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(two_sum_node src/two_sum_node.cpp)

# Modern CMake linking (replaces ament_target_dependencies)
target_link_libraries(two_sum_node PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}     # message packages export this var with their targets
)

install(TARGETS two_sum_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
