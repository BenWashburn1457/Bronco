cmake_minimum_required(VERSION 3.8)
project(bronco_description)

find_package(ament_cmake REQUIRED)

# Nothing to compile: this package only holds files. `install` copies them to
# install/bronco_description/share/bronco_description/, which is where
# $(find bronco_description) and FindPackageShare() look for them.
install(DIRECTORY urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
