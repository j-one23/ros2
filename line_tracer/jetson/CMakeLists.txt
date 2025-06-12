cmake_minimum_required(VERSION 3.16)
project(line_tracer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  /usr/local/include/dynamixel_sdk
  ${OpenCV_INCLUDE_DIRS}
)

# pub node (camera publisher with OpenCV)
add_executable(pub src/pub.cpp src/dxl.cpp)
ament_target_dependencies(pub
  rclcpp
  geometry_msgs
  sensor_msgs
  cv_bridge
)
target_link_libraries(pub dxl_x64_cpp ${OpenCV_LIBS})

# sub node
add_executable(sub src/sub.cpp src/dxl.cpp)
ament_target_dependencies(sub rclcpp geometry_msgs)
target_link_libraries(sub dxl_x64_cpp)

# main vision-based line tracer node
add_executable(line_tracer_node
  src/main.cpp
  src/vision.cpp
  src/dxl.cpp
)
ament_target_dependencies(line_tracer_node
  rclcpp
  geometry_msgs
)
target_link_libraries(line_tracer_node dxl_x64_cpp ${OpenCV_LIBS})

# Install targets
install(TARGETS
  pub
  sub
  line_tracer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
