# Description
Python firmware (highly experimental) for Neato vacuum cleaner robot and simulator

The robot is equipped with two gear motors and with 2D LiDAR sensor - this is the only sensor used in this code. The LiDAR sensor is used for mapping & localization (aka 'SLAM') and obstacle detection.

<img src="https://github.com/owlRobotics-GmbH/vacuum_cleaner/assets/11735886/b2d57bc4-9c65-4377-91f6-1766eb2d07bf" height="400" />

<img src="https://github.com/owlRobotics-GmbH/vacuum_cleaner/assets/11735886/416891d8-e6c8-41a9-867d-8b3552c1f8fe" height="400" />

# How does it work
1. Phase 'exploring': Using only the LiDAR sensor, the robot follows the walls until it gets back to it's start position.
2. Phase 'cleaning': The robot looks in the local area for free cells as targets. If there are no targets in the local area, it performs a global search for a free cell (using 'astar algorithm'). 
   
# How to install and run
Type this into your Linux terminal:
1. git clone https://github.com/owlRobotics-GmbH/vacuum_cleaner
2. pip install -r requirements.txt
3. python cleaner.py --sim    (for simulator)
4. python cleaner.py   (for Neato robot connected via serial USB)

# Where to go from here
* Tune the code
* Build your own robot with owlRobotics platform (https://github.com/owlRobotics-GmbH/owlRobotPlatform)
* Start your carreer at owlRobotics (https://owlrobotics.de/index.php/en/home-of-robotic-solutions/careers)
* ...
