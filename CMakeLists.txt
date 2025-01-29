cmake_minimum_required(VERSION 2.8.3)
project(jetson_gpio)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JETSONGPIO REQUIRED libjetsonGPIO)

# Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  InputState.msg
#  OutputState.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${JETSONGPIO_INCLUDE_DIRS}
)

add_executable(jetson_gpio_node "src/main.cpp")

target_link_libraries(jetson_gpio_node ${catkin_LIBRARIES} ${JETSONGPIO_LIBRARIES})


