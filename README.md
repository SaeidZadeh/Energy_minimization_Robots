Simulate $9$ robots on a $20\times20$ map and randomly select some positions to be explored by the robots. 
We consider robots with different maximum velocities randomly chosen from the interval $[1,2]$, with energy consumption for movement per second randomly chosen from the interval $(0,0.01)$, 
with energy consumption for exploration per second randomly chosen from the interval $(0,0.01)$, and with exploration times randomly chosen from the interval $[0.5,1]$. For the robots, 
we assume robots with differential drive and two wheels, which are initially all outside the map and arranged in a line.

The waypoints determined and then used by the robots to explore the map. We used ROS2 humble to demonstrate the result.

to reproduction of the result (ubuntu 22.04 with gazebo and ros2 installed):

- open Terminal 1 run the following:

$ source /opt/ros/humble/setup.bash

$ cd ros2_ws

$ colcon build

$ source install/setup.bash

$ ros2 launch box_bot_gazebo multi_box_bot_launch.py
 
- open Terminal 2 run the following:
 
$ source /opt/ros/humble/setup.bash

$ cd /ros2_ws/src/box_bot/box_bot_gazebo/launch

$ python3 robots_controller.py
