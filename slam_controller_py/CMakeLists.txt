cmake_minimum_required(VERSION 2.8.3)
project(slam_controller_py)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  roscpp
)
catkin_package()
include_directories( ${catkin_INCLUDE_DIRS} )

# add_dependencies(slam_controller_py ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(slam_controller_py ${catkin_LIBRARIES})
