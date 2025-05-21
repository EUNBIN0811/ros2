cmake_minimum_required(VERSION 3.8)
project(ros2_term_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 기본 패키지 및 OpenCV 의존성 추가
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)  # Occupancy Grid 사용
find_package(OpenCV REQUIRED)    # OpenCV 추가 (거리 맵 계산용)

set(ROS2_PACKAGES
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  visualization_msgs
  nav_msgs  # 추가됨
  OpenCV)   # 추가됨

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}  # OpenCV 경로 추가
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 실행 파일 추가
add_executable(cmd_publisher_node
        src/cmd_publisher_node.cpp
        src/cmd_publisher.cpp
        src/map.cpp)  # map.cpp 추가됨

ament_target_dependencies(cmd_publisher_node ${ROS2_PACKAGES})

install(TARGETS cmd_publisher_node
  DESTINATION lib/${PROJECT_NAME})

# launch 파일 설치 (필요 시)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
