
PLAN B CODE (apparently Plan A as of Feb 6)
=================================================
Overview
Idea: Robot SCANS for direction of greatest distance. Then it TRAVELS in that direction.
So the robot will be a finite state machine:
    States: 
        Scanning: Robot rotates around to find direction to travel.
        Traveling: Robot moves in straight line.
        Collision: Robot dealing with collision: robot stops immediately then moves backwards
    Transitions:
        Scanning -> Traveling: When robot decides on a direction to travel.
        Traveling -> Scanning: When robot reaches end of the path.
        Traveling -> Collision: When robot bumpers is pressed.
        Collision -> Scanning: When robot finishes moving backward
=================================================
Some useful commands.

SIMULATION ------------------------
Launching simulation bot
    roslaunch mie443_contest1 turtlebot_world.launch world:=1
Gmapping in Sim
    roslaunch turtlebot_gazebo gmapping_demo.launch
Visualizing the Gmap
roslaunch turtlebot_rviz_launchers view_navigation.launch
Running the code
rosrun mie443_contest1 contest1

REAL LIFE ---------------------------
Remote connecting
    ssh tuesday@100.65.103.84
Launching real bot
    roslaunch turtlebot_bringup minimal.launch
Gmapping
    roslaunch mie443_contest1 gmapping.launch
Visualizing the Gmap
    roslaunch turtlebot_rviz_launchers view_navigation.launch
Saving the map
    rosrun map_server map_saver -f /home/turtlebot
Running the code
    rosrun mie443_contest1 contest1 

