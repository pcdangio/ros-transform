cmake_minimum_required(VERSION 3.0.2)
project(transform)

# Find catkin an package dependenies.
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs_ext)

# Create catkin package.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES transform
  CATKIN_DEPENDS geometry_msgs_ext)

# Set up include directories.
include_directories(
  include
  ${catkin_INCLUDE_DIRS})

# Create the transform library.
add_library(${PROJECT_NAME} src/${PROJECT_NAME}/transform.cpp)

# Add build order dependency for message packages.
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Install library.
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install headers.
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp")