cmake_minimum_required(VERSION 3.5)
project(pointcloud_divider)

# Enable support for C++17
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else ()
  set(CMAKE_CXX_STANDARD 17)
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++fs")

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_package(PCL REQUIRED)

find_package(ament_cmake REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/pointcloud_divider_core.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} -lstdc++fs)

add_executable(${PROJECT_NAME}_node
  src/pointcloud_divider_core.cpp
  src/pointcloud_divider_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} -lstdc++fs)

ament_export_include_directories(include)
ament_export_dependencies(PCL)
ament_export_libraries(${PROJECT_NAME})
ament_package()

