cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs joy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs joy
  DEPENDS
)

###########
## Build ##
###########

add_executable(turtlebot_teleop_joy src/turtlebot_joy.cpp)
target_link_libraries(turtlebot_teleop_joy ${catkin_LIBRARIES})

add_executable(turtlebot_coron_vel 
  src/turtle_coron.cpp
  src/serial.cpp
  )
target_link_libraries(turtlebot_coron_vel ${catkin_LIBRARIES})

add_executable(turtlebot_coron_cont 
  src/turtle_coron_cont.cpp
  )
target_link_libraries(turtlebot_coron_cont ${catkin_LIBRARIES})


add_executable(coron_astar 
  src/turtle_coron_Astar.cpp
  )
target_link_libraries(coron_astar ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/turtlebot_teleop_key
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS turtlebot_teleop_joy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS turtlebot_coron_cont
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS turtlebot_coron_vel
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS coron_astar 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
