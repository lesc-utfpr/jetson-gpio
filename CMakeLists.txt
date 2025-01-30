cmake_minimum_required(VERSION 3.0.2)
project(jetson_gpio)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(jetson_gpio_node src/main.cpp)
target_link_libraries(jetson_gpio_node ${catkin_LIBRARIES} gpiod)
