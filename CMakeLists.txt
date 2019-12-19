cmake_minimum_required(VERSION 2.8.3)
project(sq2_ccv2)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
    roscpp
    sensor_msgs
    geometry_msgs
)

find_package(ypspur_mqtt REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES sq2_ccv2
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(ccv2_teleoperator src/ccv2_teleoperator_node.cpp src/ccv2_teleoperator.cpp)

add_dependencies(ccv2_teleoperator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ccv2_teleoperator
    mosquitto
    ${catkin_LIBRARIES}
)
