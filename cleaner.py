#!/usr/bin/env python3

'''

cleaner.py : Python firmware (highly experimental) for Neato vacuum cleaner robot and simulator

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
-------------
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

5. Further tips for manual installation:
1. conda remove -n py37 --all
2. conda create -n py37 python=3.7
3. conda activate py37
4. pip install -r requirements.txt
   OR:  
   cd BreezeSLAM/python  &&  python setup.py install
   pip install pyserial numpy matplotlib pyaml opencv-python opencv-contrib-python shapely

'''

ROBOT_DEVICE_PATH            =   '/dev/serial/by-id/usb-Linux_2.6.33.7_with_fsl-usb2-udc_Neato_Robotics_USB_v2-if00'
#ROBOT_DEVICE_PATH            = '/dev/serial/by-id/usb-Acme_Corporation_CDC_Serial_Peripheral_xxxx-xxxx-xxxx-if00'

MAP_SIZE_METERS            = 10
MAP_SCALE_METERS_PER_PIXEL = 0.05
MAP_SIZE_PIXELS            = int(MAP_SIZE_METERS / MAP_SCALE_METERS_PER_PIXEL)     # 200


MIN_SAMPLES   = 200

LIDAR_TRIGGER_TIMEOUT = 3


from shapely import geometry
import astar
import cv2
import pickle
import traceback
import time
import math
import random
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from roboviz import MapVisualizer
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import robot
import sim
import utils
import argparse


STATE_STOP         = 0 
STATE_FOLLOW_WALL  = 1
STATE_CLEAN        = 2


VAL_CLEANED     = 255   
VAL_FREE        = 63
VAL_BARRIER     = 0
VAL_BARRIER_DYN = 31
VAL_GOAL        = 200


WALL_DRIVE_RIGHT        =  0
WALL_DRIVE_SHARP_RIGHT  =  1
WALL_DRIVE_BACK         =  2
WALL_ROTATE_RECOVER     =  3


WALL_STATES = ['right', 'sharp_right', 'back', 'recover']


class Cleaner():
    def __init__(self, args):
        self.state = STATE_FOLLOW_WALL
        #self.state = STATE_CLEAN        
        self.wallState = WALL_DRIVE_RIGHT
        self.lidarTriggerTimeout = time.time() + 3.0 
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0
        self.sim = args.sim 

        print('MAP_SIZE_METERS', MAP_SIZE_METERS)
        print('MAP_SIZE_PIXELS', MAP_SIZE_PIXELS)        
        self.map_scale_meters_per_pixel = MAP_SIZE_METERS / float(MAP_SIZE_PIXELS)
        print('map_scale_meters_per_pixel', self.map_scale_meters_per_pixel)

        self.mapbytes = None
        self.mapimg = None
        self.cleanbytes = None
        self.cleanimg = None
        self.routeimg = None

        self.boundarybytes = None
        self.boundaryimg = None

        # Connect to robot (or simulator)
        if self.sim:
            self.robot = sim.Sim()                             
        else:
            self.robot = robot.Robot(ROBOT_DEVICE_PATH)                             

        self.route = []

        # Create an RMHC SLAM object with a laser model and optional robot model
        self.slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, map_quality=1)

        # Set up a SLAM display
        self.viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM', show_trajectory=False, save_figures= args.savefig)

        # Initialize an empty trajectory
        self.trajectory = []
        self.target_x = 0
        self.target_y = 0
        self.target_angle = 0
        self.targetTimeout = 0
        self.targetReached = True

        self.last_robot_x = 0
        self.last_robot_y = 0
        
        self.wallstart_robot_x = 0
        self.wallstart_robot_y = 0
        self.wallstart_dist = 0

        self.nextCheckRecoverTime = time.time() + 3.0
        self.nextVisTime = 0
        self.nextNavTime = 0
        self.shouldRecover = 0

        # Initialize empty map
        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

        # Create an iterator to collect scan data from the RPLidar
        #iterator = lidar.iter_scans()

        # We will use these to store previous scan in case current scan is inadequate
        self.previous_distances = None
        self.previous_angles    = None

        # First scan is crap, so ignore it
        #next(iterator)

        self.cleanbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.cleanimg = np.reshape(np.frombuffer(self.cleanbytes, dtype=np.uint8), (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))

        self.routebytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.routeimg = np.reshape(np.frombuffer(self.routebytes, dtype=np.uint8), (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
        #self.setRouteValue(10.0, 9.96, VAL_GOAL)

        self.wallPolygon = []

        self.nextSaveTime = 0
        self.startTime = time.time() + 3.0
        self.startTurnTime = 0
        self.nextTurnTime = 0
        self.laneCounter = 0
        self.goalAngle = 0
        self.turnLeft = False
        if self.sim and self.load():
            self.state = STATE_CLEAN



    def save(self):
        if not self.sim: return
        print('saving state...')
        print('mapbytes:', len(self.mapbytes))            
        with open('data/mapbytes.bin', 'wb') as f:  
            f.write(self.mapbytes) 
        np.save('data/cleanimg.npy', self.cleanimg)  
        np.save('data/wallpoly.npy', self.wallPolygon)  
        print('robot', self.robot_x, self.robot_y)
        with open('data/objs.pkl', 'wb') as f:  
            pickle.dump([self.robot.robot_pose[0], self.robot.robot_pose[1], self.robot.robot_pose[2]], f) 

    def load(self):
        print('loading saved state...')
        try:
            with open('data/mapbytes.bin', 'rb') as f:  
                self.mapbytes = bytearray(f.read()) 
            print('mapbytes:', type(self.mapbytes), len(self.mapbytes))
            #self.slam.setmap(self.mapbytes)                    
            self.cleanimg = np.load('data/cleanimg.npy')
            print('cleanimg:', type(self.cleanimg), len(self.cleanimg))
            self.wallPolygon = np.load('data/wallpoly.npy')            
            print('wallpoly:', type(self.wallPolygon), len(self.wallPolygon))
            with open('data/objs.pkl', 'rb') as f:  
                self.sim_x, self.sim_y, self.sim_theta = pickle.load(f) 
            print('sim', self.sim_x, self.sim_y)
            #self.robot.robot_pose[0] = self.sim_x
            #self.robot.robot_pose[1] = self.sim_y
            #self.robot.robot_pose[2] = self.sim_theta
            return True
        except:
            print('error loading saved state')
            traceback.print_exc()
            return False


    def exit(self):
        self.robot.exit()

    def flood_fill(self, map, start_position, value):
        px = int( start_position[0] / self.map_scale_meters_per_pixel)
        py = int( start_position[1] / self.map_scale_meters_per_pixel)
        start_position = (px, py)
        queue = deque([start_position])
        while queue:
            x, y = queue.popleft()
            if map[x, y] == 0:
                map[x, y] = value  # Mark as explorable
                if x > 0: queue.append((x-1, y))
                if x < map.shape[0] - 1: queue.append((x+1, y))
                if y > 0: queue.append((x, y-1))
                if y < map.shape[1] - 1: queue.append((x, y+1))


    def plotScan(self, ranges):
        plt.clf()    
        #fig = plt.figure(dpi=200)
        #ax = fig.add_subplot(projection='polar')
        angles = range(0, 360)
        #print(angles)
        angles = [angle / 180.0 * 3.1415 for angle in angles]    
        plt.polar(angles, ranges, marker='o', linestyle='')
        ax = plt.gca()
        #ax.set_ylim(0,1)    
        ax.grid(True)
        plt.pause(0.001)    
        #plt.show()


    def robotFront2Map(self, theta, distance):
        mapx = self.robot_x + distance*np.cos(self.robot_theta + theta)
        mapy = self.robot_y + distance*np.sin(self.robot_theta + theta)
        return mapx, mapy 

    def setCleanValue(self, xmeter, ymeter, value, checkBoundary = False):    
        px = int( xmeter / self.map_scale_meters_per_pixel )
        py = int( ymeter / self.map_scale_meters_per_pixel )
        #print('setCleanValue', px, py, data)    
        if checkBoundary:
            line = geometry.LineString(self.wallPolygon)
            polygon = geometry.Polygon(line)
        for x in range(-1, 2):
            for y in range(-1, 2):  
                if checkBoundary: 
                    point = geometry.Point( xmeter + x* self.map_scale_meters_per_pixel, ymeter + y* self.map_scale_meters_per_pixel)
                if (not checkBoundary) or ((x == 0) and (y == 0)) or (polygon.contains(point)):
                    self.cleanimg[py+y, px+x] = value

    def setRouteValue(self, xmeter, ymeter, value):    
        px = int( xmeter / self.map_scale_meters_per_pixel )
        py = int( ymeter / self.map_scale_meters_per_pixel )
        #print('setRouteValue', px, py, value)    
        for x in range(0, 1):
            for y in range(0, 1):  
                self.routeimg[py+y, px+x] = value

    def getCleanValue(self, xmeter, ymeter):    
        px = int( xmeter / self.map_scale_meters_per_pixel )
        py = int( ymeter / self.map_scale_meters_per_pixel )
        value = self.cleanimg[py, px] 
        return value

    def getMapValue(self, xmeter, ymeter):
        px = int( xmeter / self.map_scale_meters_per_pixel )
        py = int( ymeter / self.map_scale_meters_per_pixel )
        value = self.mapimg[py, px]
        return value
    
    def fillExplored(self):
        line = geometry.LineString(self.wallPolygon)
        polygon = geometry.Polygon(line)
        for py in range(0, MAP_SIZE_PIXELS):
            for px in range(0, MAP_SIZE_PIXELS):
                mx = px * self.map_scale_meters_per_pixel
                my = py * self.map_scale_meters_per_pixel
                value = self.getCleanValue(mx, my)
                if value == VAL_BARRIER:
                    point = geometry.Point(mx, my)
                    if polygon.contains(point):
                        self.setCleanValue(mx, my, VAL_FREE, checkBoundary=False)


    def drawRoute(self, route):
        #path = np.array( route )
        #plt.plot(path[:,1], path[:,0], linestyle="", marker="o", markerfacecolor='red')        
        #plt.show()
        self.routebytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.routeimg = np.reshape(np.frombuffer(self.routebytes, dtype=np.uint8), (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))            
        for pos in route:
            mx = pos[1] * self.map_scale_meters_per_pixel
            my = pos[0] * self.map_scale_meters_per_pixel
            self.setRouteValue(mx, my, VAL_GOAL)


    def routeToFreeCell(self):
        tx = None
        cx = int( round(self.robot_x / self.map_scale_meters_per_pixel))
        cy = int( round(self.robot_y / self.map_scale_meters_per_pixel))        
        
        mindist = 9999
        target_x = 0
        target_y = 0 
        for y in range(-10, 10):
            for x in range(-10, 10):
                px = cx + x
                py = cy + y
                if px < MAP_SIZE_PIXELS and px >= 0 and py < MAP_SIZE_PIXELS and py >= 0:                
                    if self.cleanimg[py, px] == VAL_FREE:                    
                        mx = px  * self.map_scale_meters_per_pixel
                        my = py  * self.map_scale_meters_per_pixel
                        target_angle = np.arctan2(my-self.robot_y, mx-self.robot_x)
                        headingError = utils.distancePI(self.robot_theta, target_angle)
                        #print('headingError', headingError)
                        if abs(headingError) < 0.5:
                            dist = np.sqrt( abs(mx - self.robot_x) + abs(my - self.robot_y) )
                            if dist < mindist:
                                mindist = dist
                                target_x = px 
                                target_y = py 
        if mindist < 9999:
            route = []
            route.append([target_y, target_x])
            return route 
        print('NO LOCAL CELL')       
        '''
        for y in range(MAP_SIZE_PIXELS-1,0,-1):
        #for y in range(0,MAP_SIZE_PIXELS):                    
            for x in range(0, MAP_SIZE_PIXELS):
                if self.cleanimg[y, x] == VAL_FREE:                    
                    target = astar.Node(( y, x))
                    #route.append( [y, x] )
                    #print('routeToFreeCell ', x, y)
                    #self.target_x = x * self.map_scale_meters_per_pixel
                    #self.target_y = y * self.map_scale_meters_per_pixel
                    #return route
                    break
            if not target is None: break
        if target is None: return []
        source = astar.Node((py, px))
        #print('source', source, 'target', target)
        end = astar.astar_find_target(self.cleanimg, source, target, 0)  
        #print(end.g)
        #print(Node.trace_path(end))
        return astar.Node.trace_path(end)
        '''
        s = astar.Node((cy, cx))
        #t = Node((4, 4))
        end = astar.astar_find_free(self.cleanimg, s, VAL_FREE, VAL_BARRIER)    
        if end is None: return []
        #print(end.g)
        #print(astar.Node.trace_path(end))
        return astar.Node.trace_path(end)


    def stateFollowWall(self):
        print(WALL_STATES[self.wallState],  ' dist: ', self.distLeft, self.distCenter, self.distRight, 'shouldRecover:', self.shouldRecover, 
                'wallpoly:', len(self.wallPolygon))

        if self.wallstart_robot_x != 0 and self.wallstart_robot_y != 0:
            dist = np.sqrt( (self.robot_x - self.wallstart_robot_x)**2 + (self.robot_y - self.wallstart_robot_y)**2 )  
            if self.wallstart_dist == 0:
                if dist > 1.0:
                    self.wallstart_dist = dist
            else:
                if dist < 0.5:                
                    print('floodfill')
                    self.fillExplored()
                    self.save()
                    #self.state = STATE_STOP          
                    self.state = STATE_CLEAN      
                    return          
                    #self.flood_fill( self.cleanimg, (self.robot_x, self.robot_y), VAL_FREE )

        lspd = 0.1
        aspd = 0.5
    
        headingError = utils.distancePI(self.robot_theta, self.target_angle)

        if self.wallState == WALL_DRIVE_RIGHT:
            self.robot.setSpeed(lspd, -aspd/5)            
            if time.time() > self.lidarTriggerTimeout:
                self.wallState = WALL_DRIVE_SHARP_RIGHT
                self.target_angle = self.robot_theta - 1.5
                self.targetTimeout = time.time() + 5.0
        elif self.wallState == WALL_DRIVE_SHARP_RIGHT:
            self.robot.setSpeed(lspd, -aspd)
            if abs(headingError) < 0.2 or time.time() > self.targetTimeout: 
                self.wallState = WALL_DRIVE_RIGHT
                self.lidarTriggerTimeout = time.time() + LIDAR_TRIGGER_TIMEOUT            
        elif self.wallState == WALL_DRIVE_BACK:
            self.robot.setSpeed(-lspd, aspd)
            if abs(headingError) < 0.1 or time.time() > self.targetTimeout: 
                self.wallState = WALL_DRIVE_RIGHT
                self.lidarTriggerTimeout = time.time() + LIDAR_TRIGGER_TIMEOUT
        elif self.wallState == WALL_ROTATE_RECOVER:
            self.robot.setSpeed(-lspd, aspd)
            if abs(headingError) < 0.2 or time.time() > self.targetTimeout:
                self.shouldRecover = 0
                self.nextCheckRecoverTime = time.time() + 3.0                           
                self.wallState = WALL_DRIVE_RIGHT
                self.lidarTriggerTimeout = time.time() + LIDAR_TRIGGER_TIMEOUT            

        # check if not moving anymore (needs recovery)
        if self.wallState == WALL_DRIVE_RIGHT or self.wallState == WALL_DRIVE_SHARP_RIGHT:
            if time.time() > self.nextCheckRecoverTime:
                self.nextCheckRecoverTime = time.time() + 3.0
                dist = np.sqrt( (self.robot_x - self.last_robot_x)**2 + (self.robot_y - self.last_robot_y)**2 )   
                if dist > 0.05:
                    if self.wallstart_robot_x != 0 and self.wallstart_robot_y != 0:
                        self.wallPolygon.append((self.robot_x, self.robot_y))
                    self.last_robot_x = self.robot_x
                    self.last_robot_y = self.robot_y     
                if dist < 0.05: self.shouldRecover += 1
                if self.shouldRecover > 1:
                    self.target_angle = self.robot_theta + 1.5
                    self.targetTimeout = time.time() + 5.0
                    self.wallState = WALL_ROTATE_RECOVER

        # check lidar sensor
        if self.wallState == WALL_DRIVE_RIGHT or self.wallState == WALL_DRIVE_SHARP_RIGHT:
            if self.distLeft < 150 or self.distCenter < 150 or self.distRight < 150:
                if self.wallstart_robot_x == 0 and self.wallstart_robot_y == 0:
                    self.wallstart_robot_x = self.robot_x
                    self.wallstart_robot_y = self.robot_y
                    print('wallstart:', self.wallstart_robot_x, self.wallstart_robot_y)
                self.shouldRecover = 0
                self.wallState = WALL_DRIVE_BACK
                self.target_angle = self.robot_theta + 0.2
                self.targetTimeout = time.time() + 5.0


    def stateClean(self):        
        #if time.time() < self.nextNavTime:
        #    return
        self.nextNavTime = time.time() + 1.0

        if False:
            if self.distLeft < 150 or self.distCenter < 150 or self.distRight < 150:
                mx, my = self.robotFront2Map(40/180.0 * 3.1415, 0.3)            
                self.setCleanValue(mx, my, VAL_BARRIER)                
                self.distLeft = self.getMapValue(mx, my)
                mx, my = self.robotFront2Map(0/180.0 * 3.1415, 0.3)            
                self.setCleanValue(mx, my, VAL_BARRIER)    
                self.distCenter = self.getMapValue(mx, my)
                mx, my = self.robotFront2Map(-40/180.0 * 3.1415, 0.3)                        
                self.setCleanValue(mx, my, VAL_BARRIER)    
                self.route = []
        

        dist = np.sqrt( (self.robot_x - self.target_x)**2 + (self.robot_y - self.target_y)**2 )
        if dist < 0.1: self.targetReached = True
        if self.targetReached: 
            self.targetReached = False            
            if len(self.route) == 0:
                self.route = self.routeToFreeCell() 
                print('new route: ', len(self.route))            
                self.drawRoute(self.route)
                newroute = True

            if len(self.route) == 0:
                self.state = STATE_STOP
                print('empty route')
                return

            self.setCleanValue(self.target_x, self.target_y, VAL_CLEANED, checkBoundary=True)
            py, px = self.route.pop(0) # pop front    
            #print('route px,py', px, py)         
            self.target_x = px * self.map_scale_meters_per_pixel
            self.target_y = py * self.map_scale_meters_per_pixel
            print('target pos:', self.target_x, self.target_y)            
            
        #self.setCleanValue(target_position[0], target_position[1], VAL_GOAL)

        self.target_angle = np.arctan2(self.target_y-self.robot_y, self.target_x-self.robot_x)
        # computes minimum distance between x radiant (current-value) and w radiant (set-value)
        headingError = utils.distancePI(self.robot_theta, self.target_angle)

        lspd = 0.1
        aspd = 0.5
    
        if headingError > 0.3:
            angular_speed = aspd
            linear_speed = 0
        elif headingError < -0.3:
            angular_speed = -aspd
            linear_speed = 0
        else:
            angular_speed = 0
            linear_speed = lspd

        self.robot.setSpeed(linear_speed, angular_speed)


    def run(self):
            self.robot.run()

            angles, distances = self.robot.getScanRanges() 
            angles = [int(angle / 3.1415 * 180.0) for angle in angles]                
            #plotScan(distances)
            distances = [int(distance * 1000) for distance in distances]            
            #print(distances)            
            
            # Extract (quality, angle, distance) triples from current scan
            #items = [item for item in next(iterator)]

            # Extract distances and angles from triples
            #distances = [item[2] for item in items]
            #angles    = [item[1] for item in items]

            # Update SLAM with current Lidar scan and scan angles if adequate
            if len(distances) > MIN_SAMPLES:
                self.slam.update(distances, scan_angles_degrees=angles)
                self.previous_distances = distances.copy()
                self.previous_angles    = angles.copy()

            # If not adequate, use previous
            elif self.previous_distances is not None:
                self.slam.update(self.previous_distances, scan_angles_degrees=self.previous_angles)

            # Get current robot position
            self.robot_x, self.robot_y, self.robot_thetadeg = self.slam.getpos()
            self.robot_x /= 1000.0
            self.robot_y /= 1000.0
            self.robot_theta = self.robot_thetadeg/180.0 * 3.1415

            # Get current map bytes as grayscale
            self.slam.getmap(self.mapbytes)

            #val = self.getCleanValue(self.robot_x, self.robot_y)

            # update clean map
            self.setCleanValue(self.robot_x, self.robot_y, VAL_CLEANED, checkBoundary=False)

            # look at robot front occupation data
            self.mapimg = np.reshape(np.frombuffer(self.mapbytes, dtype=np.uint8), (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
            #print(mapimg.shape)

            # Taking a matrix of size 5 as the kernel 
            kernel = np.ones((5, 5), np.uint8) 
            self.mapimg = cv2.erode(self.mapimg, kernel, iterations=1)
            self.mapimg = cv2.dilate(self.mapimg, kernel, iterations=1)            


            mx, my = self.robotFront2Map(40/180.0 * 3.1415, 0.3)            
            #setCleanValue(mx, my, 255, checkBoundary=False)                
            self.distLeft = self.getMapValue(mx, my)
            mx, my = self.robotFront2Map(0/180.0 * 3.1415, 0.3)            
            #setCleanValue(mx, my, 127, checkBoundary=False)    
            self.distCenter = self.getMapValue(mx, my)
            mx, my = self.robotFront2Map(-40/180.0 * 3.1415, 0.3)                        
            #setCleanValue(mx, my, 63, checkBoundary=False)                
            self.distRight = self.getMapValue(mx, my)


            #robot.setSpeed(0.05, -0.5)
                        
            #if True:
            if time.time() < self.startTime: 
                if self.turnLeft:
                    self.robot.setSpeed(0, 0.2)
                else:
                    self.robot.setSpeed(0, -0.2)
                if time.time() > self.startTurnTime:
                    self.startTurnTime = time.time() + 2.0
                    self.turnLeft = not self.turnLeft
                
            else:
                
                if time.time() > self.nextSaveTime:
                    self.nextSaveTime = time.time() + 10.0
                    #self.save()

                if self.state == STATE_STOP:
                    self.robot.setSpeed(0, 0)
                elif self.state == STATE_FOLLOW_WALL:
                    self.stateFollowWall()
                elif self.state == STATE_CLEAN:
                    self.stateClean()

            #self.setRouteValue(random.random()*10, random.random()*10, 255)        
            self.setRouteValue(0, 0, VAL_GOAL)        

            # Display map and robot pose, exiting gracefully if user closes it

            if time.time() > self.nextVisTime:
                self.nextVisTime = time.time() + 0.2

                if not self.viz.display(self.robot_x, self.robot_y, self.robot_thetadeg, self.mapimg, self.cleanimg, self.routeimg):
                    exit(0)

                #img = utils.alphaBlend(self.cleanimg, 125, self.routeimg, 125)
                #img = cv2.resize(img, (0,0), fx=8.0, fy=8.0, interpolation = cv2.INTER_NEAREST) 
                #cv2.imshow("cleaner", img)
                #cv2.waitKey(1)


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description='cleaner')    
        parser.add_argument('-s','--sim', action='store_true', help='simulator',required=False)    
        parser.add_argument('-f','--savefig', action='store_true', help='save figures',required=False)    
        args = parser.parse_args()

        cleaner = Cleaner(args)
        while True:
            cleaner.run()

    except Exception:
        traceback.print_exc()
    finally:        
        # Shut down the lidar connection
        #lidar.stop()
        #lidar.disconnect()
        cleaner.exit()
