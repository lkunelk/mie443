# robobois
Some useful commands for Contest 2.

## Simulation
```
catkin_make
roslaunch mie443_contest2 turtlebot_world.launch world:=1
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/<absolute_map_path>/map_1.yaml
rosrun mie443_contest2 contest2
```

### Robot Execution (AMCL)
```
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/<absolute_map_path>/map1.yaml
```

Ensure Turtlebot is properly connected to the laptop and amcl is running
```
roslaunch turtlebot_rviz_launchers view_navigation.launch
rostopic echo /amcl_pose 
rosrun mie443_contest2 contest2
```

### Subscribe to Webcam instead of Kinect
Change IMAGE_TOPIC in imagePipeline.cpp

```
rosrun mie443_contest2 webcam_publisher 0
```