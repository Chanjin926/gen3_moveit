
cmake_minimum_required(VERSION 2.8.3)
project(gen3_moveit)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation actionlib_msgs actionlib kortex_driver moveit_ros_planning_interface moveit_visual_tools)
find_package(Eigen3 REQUIRED)
## Declare a catkin package
catkin_package()

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)

add_executable(pick_place src/move_it/pick_place.cpp)
add_dependencies(pick_place ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_place ${catkin_LIBRARIES} )



# Install


## Mark executables and/or libraries for installation
install(TARGETS 
    pick_place
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
