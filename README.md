# vacuum_cleaner
Python firmware (highly experimental) for Neato vacuum cleaner robot and simulator

The robot is equipped with two gear motors and with 2D LiDAR sensor - this is the only sensor used in this code. The LiDAR sensor is used for mapping & localization (aka 'SLAM') and obstacle detection.

1. How does it work
Phase 1 ('exploring'): Using only the LiDAR sensor, the robot follows the walls until it gets back to it's start position.
Phase 2 ('cleaning'): The robot looks in the local area for free cells as targets. If there are no targets in the local area, it performs a global search for a free cell (using 'astar algorithm'). 
   
3. How to install and run
git clone https://github.com/owlRobotics-GmbH/vacuum_cleaner
pip install -r requirements.txt
python cleaner.py

4. Where to go from here
* Build your own robot with owlRobotics platform (https://github.com/owlRobotics-GmbH/owlRobotPlatform)
* Start your carreer at owlRobotics (https://owlrobotics.de/index.php/en/home-of-robotic-solutions/careers)
* ...
