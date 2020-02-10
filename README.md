# robobois
Some useful commands.

### SIMULATION
```
roslaunch mie443_contest1 turtlebot_world.launch world:=1
roslaunch turtlebot_gazebo gmapping_demo.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch
rosrun mie443_contest1 contest1
```

### REAL LIFE
```
ssh tuesday@100.65.103.84 //remote connect to our computer
roslaunch turtlebot_bringup minimal.launch
roslaunch mie443_contest1 gmapping.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch
rosrun mie443_contest1 contest1
```

Saving the map
```
rosrun map_server map_saver -f /home/turtlebot
```
