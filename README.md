Simulate $9$ robots on a $20\times20$ map and randomly select some positions to be explored by the robots. 
We consider robots with different maximum velocities randomly chosen from the interval $[1,2]$, with energy consumption for movement per second randomly chosen from the interval $(0,0.01)$, 
with energy consumption for exploration per second randomly chosen from the interval $(0,0.01)$, and with exploration times randomly chosen from the interval $[0.5,1]$. For the robots, 
we assume robots with differential drive and two wheels, which are initially all outside the map and arranged in a line.

The waypoints determined and then used by the robots to explore the map. We used ROS2 humble to demonstrate the result.
