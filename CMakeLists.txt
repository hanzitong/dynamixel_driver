cmake_minimum_required(VERSION 3.8)
project(dynamixel_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
## memo: install dynamixel_sdk to /usr/lib & /usr/local/include

include_directories(
  include
  ${dynamixel_driver_INCLUDE_DIRS}
)

add_library(dxl_driver_lib src/DxlXDriver.cpp)
ament_target_dependencies(dxl_driver_lib
    rclcpp
    geometry_msgs
)
target_link_libraries(dxl_driver_lib dxl_x64_cpp)


add_executable(dxl_x_driver src/dxl_x_driver.cpp)
ament_target_dependencies(dxl_x_driver
    rclcpp
)
target_link_libraries(dxl_x_driver dxl_driver_lib)

install(TARGETS
    dxl_x_driver
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
