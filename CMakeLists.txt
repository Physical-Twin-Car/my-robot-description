cmake_minimum_required(VERSION 3.5)
project(my_robot_description)

# Compiler Options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
#find_package(<dependency> REQUIRED) add more here

# Install files
install(
  DIRECTORY config description launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)


ament_package()