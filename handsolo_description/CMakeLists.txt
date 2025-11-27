cmake_minimum_required(VERSION 3.8)
project(handsolo_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY xacro 
    DESTINATION share/${PROJECT_NAME})