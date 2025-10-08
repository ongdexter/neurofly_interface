cmake_minimum_required(VERSION 3.10)
project(neurofly_system_launch)

# Use ament for ROS 2 packages
find_package(ament_cmake REQUIRED)

# Install launch files to the package share directory
install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()