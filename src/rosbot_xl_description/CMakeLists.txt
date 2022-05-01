cmake_minimum_required(VERSION 3.5)
project(rosbot_xl_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED
  rosbot_xl_controllers
  pluginlib)

install(
  DIRECTORY rviz models config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()