# citylab_project | Ros2 with Turtlebot3
This work include topics, service and action code in ROS2.

## Mandatory
+ For compiling
```
cd ~/ros2_ws/ ;colcon build ;source install/setup.bash
cd ~/ros2_ws/ ;colcon build --packages-select custom_interface; source install/setup.bash
cd ~/ros2_ws/ ;colcon build --packages-select robot_patrol; source install/setup.bash
```
+ For sourcing only
```
cd ~/ros2_ws/ ;source install/setup.bash
```

## Tasks
- [x] topics 
```
ros2 launch robot_patrol start_patrolling.launch.py
```
![patrol](https://github.com/Andy-Leo10/citylab_project/assets/60716487/360d1818-d246-4f7a-9884-0c85564c95d6)

- [x] service 
```
ros2 launch robot_patrol start_direction_service.launch.py
ros2 launch robot_patrol start_test_service.launch.py
ros2 launch robot_patrol main.launch.py
```
![service](https://github.com/Andy-Leo10/citylab_project/assets/60716487/19c19b71-4201-4d0a-be95-561ace8e2cc3)
![main](https://github.com/Andy-Leo10/citylab_project/assets/60716487/a2d702a2-c327-4f82-afe8-53b372142159)

- [x] action 
```
ros2 launch robot_patrol start_gotopose_action.launch.py
ros2 action send_goal -f /go_to_pose custom_interface/action/GoToPose "goal_pos: x: 0.2 y: 0.3 theta: 40.0"
```
![action](https://github.com/Andy-Leo10/citylab_project/assets/60716487/458bee14-f0f9-4d88-abaa-47358c9ab5dc)

## Annexes
* Simulation
```
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash
roslaunch realrobotlab main.launch
```
* Bridge
```
source ~/catkin_ws/devel/setup.bash
roslaunch load_params load_params.launch
source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge parameter_bridge
```

## Other for testing
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
