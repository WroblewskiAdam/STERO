cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED roscpp std_msgs geometry_msgs nav_msgs tf2_ros global_planner clear_costmap_recovery move_base base_local_planner nav_core tf2_geometry_msgs clear_costmap_recovery rotate_recovery)

catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp nodelet
#   DEPENDS elektron
)

include_directories(include ${catkin_INCLUDE_DIRS})

install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
install (DIRECTORY param/ DESTINATION share/${CMAKE_PROJECT_NAME}/param 
         FILES_MATCHING PATTERN "*.yaml")

add_executable(my_move_base src/my_move_base.cpp)
target_link_libraries(my_move_base ${catkin_LIBRARIES})
#add_dependencies(my_move_base stero)

add_executable(lab1 src/lab1.cpp)
target_link_libraries(lab1 ${catkin_LIBRARIES})
#add_dependencies(lab1 stero)

add_executable(error_calculation src/error_calculation.cpp)
target_link_libraries(error_calculation ${catkin_LIBRARIES})
#add_dependencies(error_calculation stero)
