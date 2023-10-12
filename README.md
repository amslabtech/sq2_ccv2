# sq2_ccv2
ROS package and parameter files for SQ2-CCV2

# Dependencies
- [ypspur_mqtt](https://github.com/amslabtech/ypspur_mqtt)
- [ccv_dynamixel_controller](https://github.com/amslabtech/ccv_dynamixel_controller)

# Build
```
cd ~/catkin_ws/src
git clone https://github.com/amslabtech/sq2_ccv2
rosdep install -riy --from-paths .
catkin build -DCMAKE_BUILD_TYPE=Release
```
