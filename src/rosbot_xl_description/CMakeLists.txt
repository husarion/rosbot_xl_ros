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
  DIRECTORY rviz models launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_package()