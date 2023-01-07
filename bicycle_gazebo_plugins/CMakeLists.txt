cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#Set project name
project(bicycle_gazebo_plugins)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  std_msgs
  bicycle_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(./include ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

#add_library(${PROJECT_NAME} src/bicycle_plugin.cpp src/pid_controller.cpp src/test_plugin.cpp)
#add_library(${PROJECT_NAME}  src/test_plugin.cpp)
add_library(test_plugin  src/test_plugin.cpp)
add_library(bicycle_plugin  src/bicycle.cpp)

# Need to link to gazebo_ros_utils.so. CLEAN THIS UP
#target_link_libraries(${PROJECT_NAME}  /opt/ros/$ENV{ROS_DISTRO}/lib/libgazebo_ros_utils.so ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(test_plugin  ${roslib_LIBRARY_DIRS}/libgazebo_ros_utils.so ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(bicycle_plugin  ${roslib_LIBRARY_DIRS}/libgazebo_ros_utils.so ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#Outputted to /home/diehl/catkin_ws/devel/lib/<library name>


macro(print_all_variables)
    message(STATUS "print_all_variables------------------------------------------{")
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
    message(STATUS "print_all_variables------------------------------------------}")
endmacro()
print_all_variables()