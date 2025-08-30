cmake_minimum_required(VERSION 3.20)
project(localization-app CXX)

add_executable(app lane_align_task.cpp)

# vcpkg provides the config package:
find_package(OpenCV REQUIRED)
target_link_libraries(app PRIVATE ${OpenCV_LIBS})
target_include_directories(app PRIVATE ${OpenCV_INCLUDE_DIRS})
