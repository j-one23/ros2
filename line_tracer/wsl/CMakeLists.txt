cmake_minimum_required(VERSION 3.16)
project(line_tracer)
 # Default to C++14
if(NOTCMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 # find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(include)
include_directories(/usr/local/include/dynamixel_sdk)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
 # Build
add_executable(pub src/pub.cpp )
ament_target_dependencies(pub rclcpp geometry_msgs)
ament_target_dependencies(pub rclcpp sensor_msgs cv_bridge OpenCV)
#target_link_libraries(pub dxl_x64_cpp)
#add_executable(sub src/sub.cpp src/dxl.cpp)
#ament_target_dependencies(sub rclcpp geometry_msgs)
#target_link_libraries(sub dxl_x64_cpp)
install(TARGETS
  pub
  DESTINATION lib/${PROJECT_NAME})
ament_package()