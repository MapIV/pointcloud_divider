cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_divider)

# Enable support for C++14
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else ()
  set(CMAKE_CXX_STANDARD 14)
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++fs")

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  INCLUDE_DIRS include ${PCL_INCLUDE_DIRS}
  DEPENDS PCL
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/pointcloud_divider_core.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} -lstdc++fs)

add_executable(${PROJECT_NAME}_node
  src/pointcloud_divider_core.cpp
  src/pointcloud_divider_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} -lstdc++fs)
