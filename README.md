# lidar_body_tracking
ROS Catkin package to track people using ortree and cluster extraction from a fixed point.  
Sensor Used for testing: Quanergy M8  
Written and tested on: Ubuntu 16.04, ROS Kinetic  
![](images/lidar_tracking.gif)  
![](images/clustered_markers.gif)  

## Dependencies  
This package depends on the people_msgs package:  
![kdhansen/people](https://github.com/kdhansen/people)  
1. `sudo apt install ros-kinetic-easy-markers ros-kinetic-kalman-filter`  
2. `git clone https://github.com/kdhansen/people ~/catkin_ws/src`  

## Installation
1. Make your catkin workspace:
    1. `mkdir -p ~/catkin_ws/src`
    2. `catkin_init_workspace ~/catkin_ws/src`
    3. `catkin_make -C ~/catkin_ws`
2. Clone `kdhansen/people` to your workspace:  
    1. `git clone https://github.com/kdhansen/people ~/catkin_ws/src`  
3. Clone this repo to your workspace:
    1. `git clone https://github.com/MyNameIsCosmo/lidar_body_tracking ~/catkin_ws/src`
4. Source your workspace
    1. `source ~/catkin_ws/devel/setup.sh`
5. Build your workspace  
    1. `catkin_make -C ~/catkin_ws --pkg people_msgs`  
    2. `catkin_make -C ~/catkin_ws`

## Running the tracking
1. Initialize your LIDAR, or play your ROSBAG  
    1. `rosbag play -l bagname.bag`
2. Launch 
    1. `source ~/catkin_ws/devel/setup.sh`
    2. `roslaunch lidar_body_tracking lidar_body_tracking.launch`

## Notes
1. The URDF frame is QP308. You can change this in `/urdf/m8.launch.xacro`  
2. You can change the default topics in the launch file `/launch/lidar_body_tracking.launch`  

## TODO:
- [x] Dynamic Reconfigure for Link, Leaf size, min cluster, etc  
- [x] ROSParam for topics
- [x] Clustering of indicies for person detection  
- [ ] Output potential people to a topic  
    - [ ] Calculate person velocity
    - [ ] Calculate certainty of person
    - [ ] Estimate person height, size
    - [ ] Track person based on previous location  
    - [ ] people_msgs/Person does not include orientation  
- [x] Control an RVIZ marker or something  
- [ ] Support body tracking while lidar is moving, loop closure and Odom tracking.  
- [ ] Comment and document code  
    - [ ] Object-oriented
    - [ ] Clean-up code

## References
[ROS WIKI URDF](http://wiki.ros.org/urdf)  
[ROS WIKI Xacro Reference](http://wiki.ros.org/xacro)  
