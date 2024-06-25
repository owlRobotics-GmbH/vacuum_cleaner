"""
actual robot  (uses Neato robot driver to control motors, read LiDAR sensor etc.)

Copyright (C) 2024 Alexander Grau

This file is part of VacuumCleaner.

VacuumCleaner is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

VacuumCleaner is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

"""

import sys
import time
import numpy as np
import matplotlib.pylab as pl
import yaml
from obstacle import Obstacle
#from moviepy.editor import VideoClip #moviepy v. 0.2.2.11
import neato_driver



class Robot():


    def __init__(self, aRobotDevicePath):
        self.robotDevicePath = aRobotDevicePath
        self.ranges = []
        self.angles = []
        self.robot_linear_speed = 0.0        
        self.robot_angular_speed = 0.0
        self.lastRunTime = time.time()
        # choose a robot software driver (Neato etc.)
        self.driver = neato_driver.xv11(self.robotDevicePath)
        self.driver.setTestMode("on")
        self.driver.setLDS("on")

        
    def setSpeed(self, linearSpeed, angularSpeed):
        self.robot_linear_speed = linearSpeed
        self.robot_angular_speed = angularSpeed

        x = self.robot_linear_speed * 1000
        th = self.robot_angular_speed * (neato_driver.BASE_WIDTH/2) 
        k = max(abs(x-th),abs(x+th))
        # sending commands higher than max speed will fail
        if k > neato_driver.MAX_SPEED:
            x = x*neato_driver.MAX_SPEED/k; th = th*neato_driver.MAX_SPEED/k
        leftSpeed = int(x-th) 
        rightSpeed = int(x+th)
        
        # send updated movement commands
        self.driver.setMotors(leftSpeed, rightSpeed, max(abs(leftSpeed),abs(rightSpeed))) 



    # angles in radiant, ranges in meter
    def getScanRanges(self):
        return self.angles, self.ranges 


    def run(self):
        self.driver.requestScan()            
        self.ranges = self.driver.getScanRanges() 
        self.angles = list(range(0, 360))    
        #print('angles', angles)
        self.angles = [angle / 180.0 * 3.1415 for angle in self.angles]    
        self.ranges = list(self.ranges[180:]) + list(self.ranges[:180])  # rotate 

    
    def exit(self):
        self.driver.exit()
        
    