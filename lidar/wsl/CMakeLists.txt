cmake_minimum_required(VERSION 3.5)
project(lidarplot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

set(SLLIDAR_SDK_PATH "./sdk/")

file(GLOB SLLIDAR_SDK_SRC 
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)  # ✅ OpenCV 추가

################################################################################
# Include directories
################################################################################
include_directories(
  ${SLLIDAR_SDK_PATH}/include
  ${SLLIDAR_SDK_PATH}/src
  ${OpenCV_INCLUDE_DIRS}       # ✅ OpenCV include 경로 추가
)

################################################################################
# Build Nodes
################################################################################

add_executable(lidarplot src/lidarplot.cpp)
ament_target_dependencies(lidarplot
  rclcpp
  std_srvs
  sensor_msgs
  OpenCV
)

target_link_libraries(lidarplot ${OpenCV_LIBS})  # ✅ OpenCV 연결

################################################################################
# Install
################################################################################

install(
  TARGETS  lidarplot
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Export
################################################################################
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_package()
