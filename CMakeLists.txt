cmake_minimum_required(VERSION 2.8.3)
project(robot_model_to_pointcloud)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
    moveit_core 
    moveit_ros_planning_interface 
    geometric_shapes
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
 
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
