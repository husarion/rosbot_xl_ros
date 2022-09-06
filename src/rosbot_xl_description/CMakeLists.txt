cmake_minimum_required(VERSION 3.10.2)
project(rosbot_xl_description)


find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()