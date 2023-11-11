# sq2_ccv2
ROS package and parameter files for SQ2-CCV2

# Dependencies
- [MqttClass](https://github.com/amslabtech/MqttClass)
- [ypspur_mqtt](https://github.com/amslabtech/ypspur_mqtt)
- [imu_subscriber](https://github.com/amslabtech/imu_subscriber)
- [ccv_dynamixel_controller](https://github.com/amslabtech/ccv_dynamixel_controller)

# Build
```
cd ~/catkin_ws/src
git clone https://github.com/amslabtech/sq2_ccv2
rosdep install -riy --from-paths .
catkin build -DCMAKE_BUILD_TYPE=Release
```
