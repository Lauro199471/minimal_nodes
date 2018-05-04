cmake_minimum_required(VERSION 2.8.3)
project(minimal_nodes)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)
FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi /opt/wiringPi)

add_message_files(
  FILES
  ads1115_sensorVal.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${WIRINGPI_LIBRARY})

## Declare a cpp executable
add_executable(minimal_publisher src/minimal_publisher.cpp)
add_executable(minimal_subscriber src/minimal_subscriber.cpp)
add_executable(minimal_simulator src/minimal_simulator.cpp)
add_executable(minimal_controller src/minimal_controller.cpp)
add_executable(minimal_wiringPi src/minimal_wiringPi.cpp)
add_executable(minimal_pwm src/minimal_pwm.cpp)
add_executable(xbox_to_pwm src/xbox_to_pwm.cpp)
add_executable(minimal_ads1115 src/minimal_ads1115.cpp)
add_executable(minimal_adxl345 src/minimal_adxl345.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(minimal_wiringPi minimal_nodes_generate_messages_cpp wiringPi)
add_dependencies(minimal_pwm minimal_nodes_generate_messages_cpp wiringPi)
add_dependencies(xbox_to_pwm minimal_nodes_generate_messages_cpp wiringPi)
add_dependencies(minimal_ads1115 minimal_nodes_generate_messages_cpp wiringPi)
add_dependencies(minimal_adxl345 minimal_nodes_generate_messages_cpp wiringPi)

## Specify libraries to link a library or executable target against
target_link_libraries(minimal_publisher  ${catkin_LIBRARIES} )
target_link_libraries(minimal_subscriber  ${catkin_LIBRARIES} )
target_link_libraries(minimal_simulator  ${catkin_LIBRARIES} )
target_link_libraries(minimal_controller  ${catkin_LIBRARIES} )
target_link_libraries(minimal_wiringPi  ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
target_link_libraries(minimal_pwm  ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
target_link_libraries(xbox_to_pwm  ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
target_link_libraries(minimal_ads1115  ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
target_link_libraries(minimal_adxl345  ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})



