cmake_minimum_required(VERSION 3.5)
project(pointcloud_divider)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(map4_point_type REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/pointcloud_divider_core.cpp)
ament_target_dependencies(${PROJECT_NAME}
  map4_point_type
  yaml_cpp_vendor
)

add_executable(${PROJECT_NAME}_node
  src/pointcloud_divider_core.cpp
  src/pointcloud_divider_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  map4_point_type
  yaml_cpp_vendor
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(map4_point_type)
ament_export_dependencies(yaml_cpp_vendor)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()