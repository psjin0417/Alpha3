cmake_minimum_required(VERSION 3.0.2)
project(package)

find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_ros
  tf2_geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation  # 커스텀 메시지 활용을 위해 필수적으로 추가
  morai_msgs
)

# ========== 커스텀 메시지 사용 시 ==========

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  src  # 헤더파일이 src 폴더 내에 있으므로
)

add_message_files(
    FILES
    gnss.msg
    object.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # 사용할 다른 패키지의 메시지 타입을 나열
)
# ========================================

# ======== 현재 패키지가 의존하는 다른 ROS 패키지 명시 ========
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime morai_msgs tf2 tf2_ros tf2_geometry_msgs
)
# ==========================================================

# GeographicLib 추가
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

# include 디렉토리에서 헤더 파일 가져올 때 include 주석 제거
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}  # GeographicLib 포함
)

# ================== Cpp 파일 제작 시 ==================
add_executable(makepath src/makepath.cpp)
target_link_libraries(makepath
    ${catkin_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libGeographic.so
)

add_executable(gps src/gps.cpp)
target_link_libraries(gps
    ${catkin_LIBRARIES}
)

add_executable(main src/main.cpp)
target_link_libraries(main
    ${catkin_LIBRARIES}
)
# ===================================================

# ==================== 라이브러리 제작 ====================
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/package.cpp
# )
# ===================================================
