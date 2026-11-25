cmake_minimum_required(VERSION 3.8)
project(bronco_gazebo)

find_package(ament_cmake REQUIRED)

# Only data files (no C++ to compile): install them so launch files can find them.
install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
