cmake_minimum_required(VERSION 3.5)
project(esp32_uwb)

## Use C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

## Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(Boost REQUIRED)

## Specify additional locations of header files
include_directories(
    include
    # ${EIGEN3_INCLUDE_DIRS}
    # ${Boost_INCLUDE_DIRS}
)

## Dependencies
set(DEPENDENCIES
    rclcpp
    std_msgs
    serial
)

add_executable(esp32_uwb_publisher src/esp32_uwb_publisher.cpp)
# target_include_directories(esp32_uwb_publisher PUBLIC include)
ament_target_dependencies(esp32_uwb_publisher ${DEPENDENCIES})

install(TARGETS esp32_uwb_publisher
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)

ament_package()