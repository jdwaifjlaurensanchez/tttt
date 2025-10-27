# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 3.0.2)
project(catkin_ws)

# Load catkin macros and environment
find_package(catkin REQUIRED)

# Enable catkin workspace
catkin_workspace()