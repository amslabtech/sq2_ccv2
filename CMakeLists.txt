cmake_minimum_required(VERSION 2.8.3)
project(sq2_ccv2)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    ccv_dynamixel_msgs
    tf2
    dynamixel_workbench_msgs
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

catkin_install_python(PROGRAMS scripts/set_cams_transforms.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(ccv2_teleoperator src/ccv2_teleoperator_node.cpp src/ccv2_teleoperator.cpp)
add_dependencies(ccv2_teleoperator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ccv2_teleoperator
    mosquitto
    ${catkin_LIBRARIES}
)

add_executable(odom_converter src/odom_converter_node.cpp src/odom_converter.cpp)
add_dependencies(odom_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_converter
    mosquitto
    ${catkin_LIBRARIES}
)

add_executable(imu_converter src/imu_converter_node.cpp src/imu_converter.cpp)
add_dependencies(imu_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_converter
    mosquitto
    ${catkin_LIBRARIES}
)

add_executable(imu_to_deg src/imu_to_deg.cpp)
add_dependencies(imu_to_deg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_to_deg ${catkin_LIBRARIES})
