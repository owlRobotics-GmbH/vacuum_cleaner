"""
robot simulator (controls simulated robot, generates laser reflections in dynamic environments)

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


BASE_WIDTH = 248    # millimeters
MAX_SPEED = 300     # millimeters/second




def connect_segments(segments, resolution = 0.01):
    """
    :param segments: start and end points of all segments as ((x1,y1,x1',y1'), (x2,y2,x2',y2'), (x3,y3,x3',y3'), (...))
           step_size : resolution for plotting
    :return: stack of all connected line segments as (X, Y)
    """

    for i, seg_i in enumerate(segments):
        if seg_i[1] == seg_i[3]: #horizontal segment
            x = np.arange(min(seg_i[0],seg_i[2]), max(seg_i[0],seg_i[2]), resolution)
            y = seg_i[1]*np.ones(len(x))
        elif seg_i[0] == seg_i[2]: #vertical segment
            y = np.arange(min(seg_i[1],seg_i[3]), max(seg_i[1],seg_i[3]), resolution)
            x = seg_i[0]*np.ones(len(y))
        else: # gradient exists
            m = (seg_i[3] - seg_i[1])/(seg_i[2] - seg_i[0])
            c = seg_i[1] - m*seg_i[0]
            x = np.arange(min(seg_i[0],seg_i[2]), max(seg_i[0],seg_i[2]), resolution)
            y = m*x + c

        obs = np.vstack((x, y)).T
        if i == 0:
            connected_segments = obs
        else:
            connected_segments = np.vstack((connected_segments, obs))

    return connected_segments

def get_intersection(a1, a2, b1, b2) :
    """
    :param a1: (x1,y1) line segment 1 - starting position
    :param a2: (x1',y1') line segment 1 - ending position
    :param b1: (x2,y2) line segment 2 - starting position
    :param b2: (x2',y2') line segment 2 - ending position
    :return: point of intersection, if intersect; None, if do not intersect
    #adopted from https://github.com/LinguList/TreBor/blob/master/polygon.py
    """
    def perp(a) :
        b = np.empty_like(a)
        b[0] = -a[1]
        b[1] = a[0]
        return b

    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot( dap, db)
    num = np.dot( dap, dp )

    intersct = np.array((num/denom.astype(float))*db + b1) #TODO: check divide by zero!

    delta = 1e-3
    condx_a = min(a1[0], a2[0])-delta <= intersct[0] and max(a1[0], a2[0])+delta >= intersct[0] #within line segment a1_x-a2_x
    condx_b = min(b1[0], b2[0])-delta <= intersct[0] and max(b1[0], b2[0])+delta >= intersct[0] #within line segment b1_x-b2_x
    condy_a = min(a1[1], a2[1])-delta <= intersct[1] and max(a1[1], a2[1])+delta >= intersct[1] #within line segment a1_y-b1_y
    condy_b = min(b1[1], b2[1])-delta <= intersct[1] and max(b1[1], b2[1])+delta >= intersct[1] #within line segment a2_y-b2_y
    if not (condx_a and condy_a and condx_b and condy_b):
        intersct = None #line segments do not intercept i.e. interception is away from from the line segments

    return intersct

def get_laser_ref(segments, fov=np.pi, n_reflections=180, max_dist=100, xytheta_robot=np.array([0.0, 0.0])):
    """
    :param segments: start and end points of all segments as ((x1,y1,x1',y1'), (x2,y2,x2',y2'), (x3,y3,x3',y3'), (...))
           fov: sight of the robot - typically pi or 4/3*pi
           n_reflections: resolution=fov/n_reflections
           max_dist: max distance the robot can see. If no obstacle, laser end point = max_dist
           xy_robot: robot's position in the global coordinate system
    :return: 1xn_reflections array indicating the laser end point
    """
    xy_robot = xytheta_robot[:2] #robot position
    theta_robot = xytheta_robot[2] #robot angle in rad

    angles = np.linspace(theta_robot, theta_robot+fov, n_reflections)
    dist_theta = max_dist*np.ones(n_reflections) # set all laser reflections to 100

    for seg_i in segments:
        xy_i_start, xy_i_end = np.array(seg_i[:2]), np.array(seg_i[2:]) #starting and ending points of each segment
        for j, theta in enumerate(angles):
            xy_ij_max = xy_robot + np.array([max_dist*np.cos(theta), max_dist*np.sin(theta)]) # max possible distance
            intersection = get_intersection(xy_i_start, xy_i_end, xy_robot, xy_ij_max)

            if intersection is not None: #if the line segments intersect
                r = np.sqrt(np.sum((intersection-xy_robot)**2)) #radius

                if r < dist_theta[j]:
                    dist_theta[j] = r

    return dist_theta

def get_laser_ref_covered(segments, fov=np.pi, n_reflections=180, max_dist=100, xytheta_robot=np.array([0.0, 0.0])):
    """
    :param segments: start and end points of all segments as ((x1,y1,x1',y1'), (x2,y2,x2',y2'), (x3,y3,x3',y3'), (...))
           fov: sight of the robot - typically pi or 4/3*pi
           n_reflections: resolution=fov/n_reflections
           max_dist: max distance the robot can see. If no obstacle, laser end point = max_dist
           xy_robot: robot's position in the global coordinate system
    :return: 1xn_reflections array indicating the laser end point
    """

    covered_segments = segments[:2]
    print(segments)
    sys.exit()

    xy_robot = xytheta_robot[:2] #robot position
    theta_robot = xytheta_robot[2] #robot angle in rad

    angles = np.linspace(theta_robot, theta_robot+fov, n_reflections)
    dist_theta = max_dist*np.ones(n_reflections) # set all laser reflections to 100

    for seg_i in segments:
        xy_i_start, xy_i_end = np.array(seg_i[:2]), np.array(seg_i[2:]) #starting and ending points of each segment
        for j, theta in enumerate(angles):
            xy_ij_max = xy_robot + np.array([max_dist*np.cos(theta), max_dist*np.sin(theta)]) # max possible distance
            intersection = get_intersection(xy_i_start, xy_i_end, xy_robot, xy_ij_max)

            if intersection is not None: #if the line segments intersect
                r = np.sqrt(np.sum((intersection-xy_robot)**2)) #radius

                if r < dist_theta[j]:
                    dist_theta[j] = r

    return dist_theta

def get_way_points_gui(environment, vehicle_poses=None):
    """
    :param environment: yaml config file
    :param vehicle_poses: vehicle poses
    :return:
    """
    class mouse_events:
        def __init__(self, fig, line):
            self.path_start = False #If true, capture data
            self.fig = fig
            self.line = line
            self.xs = list(line.get_xdata())
            self.ys = list(line.get_ydata())
            self.orientation = []

        def connect(self):
            self.a = self.fig.canvas.mpl_connect('button_press_event', self.__on_press)
            self.b = self.fig.canvas.mpl_connect('motion_notify_event', self.__on_motion)

        def __on_press(self, event):
            print('You pressed', event.button, event.xdata, event.ydata)
            self.path_start = not self.path_start

        def __on_motion(self, event):
            if self.path_start is True:
                if len(self.orientation) == 0:
                    self.orientation.append(0)
                else:
                    self.orientation.append(np.pi/2 + np.arctan2((self.ys[-1] - event.ydata), (self.xs[-1] - event.xdata)))
                self.xs.append(event.xdata)
                self.ys.append(event.ydata)
                self.line.set_data(self.xs, self.ys)
                self.line.figure.canvas.draw()

    # set up the environment
    all_obstacles, area = load_obstacles_config(environment=environment)

    # update obstacles
    all_obstacle_segments = []
    for obs_i in all_obstacles:
        all_obstacle_segments += obs_i.update()

    connected_components = connect_segments(all_obstacle_segments)

    # plot
    pl.close('all')
    fig = pl.figure()#figsize=(10, 5))  # (9,5)
    ax = fig.add_subplot(111)
    pl.title('Generate waypoints: 1) Click to start. 2) Move the mouse. \n3) Click to stop. 4) lose the gui to exit')
    ax.scatter(connected_components[:, 0], connected_components[:, 1], marker='.', c='y', edgecolor='', alpha=0.2)  # obstacles
    if vehicle_poses is not None:
        pl.plot(vehicle_poses[:, 0], vehicle_poses[:, 1], 'o--', c='m')
    pl.xlim(area[:2]); pl.ylim(area[2:])

    line, = ax.plot([], [])
    mouse = mouse_events(fig, line)
    mouse.connect()

    pl.show()

    return np.hstack((np.array(mouse.xs)[:, None], np.array(mouse.ys)[:, None], np.array(mouse.orientation)[:,None]))[1:]

def get_filled_txy(dist_theta, robot_pos, fov, n_reflections, max_laser_distance, unoccupied_points_per_meter=0.1, margin=0.1):
    """
    :param dist_theta: lidar hit distance
    :param robot_pos: robot pose
    :param fov: robot field of view
    :param n_reflections: number of lidar hits
    :param max_laser_distance: maximum lidar distance
    :param unoccupied_points_per_meter: in-fill density
    :param margin: in-fill density of free points
    :return: (points, labels) - 0 label for free points and 1 label for hits
    """

    angles = np.linspace(robot_pos[2], robot_pos[2] + fov, n_reflections)
    laser_data_xy = np.vstack([dist_theta * np.cos(angles), dist_theta * np.sin(angles)]).T + robot_pos[:2]

    for i, ang in enumerate(angles):
        dist = dist_theta[i]
        laser_endpoint = laser_data_xy[i,:]

        # parametric filling
        para = np.sort(np.random.random(np.int16(dist * unoccupied_points_per_meter)) * (1 - 2 * margin) + margin)[:, np.newaxis]  # TODO: Uniform[0.05, 0.95]
        points_scan_i = robot_pos[:2] + para*(laser_endpoint - robot_pos[:2])  # y = <x0, y0, z0> + para <x, y, z>; para \in [0, 1]

        if i == 0:  # first data point
            if dist >= max_laser_distance:  # there's no laser reflection
                points = points_scan_i
                labels = np.zeros((points_scan_i.shape[0], 1))
            else:  # append the arrays with laser end-point
                points = np.vstack((points_scan_i, laser_endpoint))
                labels = np.vstack((np.zeros((points_scan_i.shape[0], 1)), np.array([1])[:, np.newaxis]))
        else:
            if dist >= max_laser_distance:  # there's no laser reflection
                points = np.vstack((points, points_scan_i))
                labels = np.vstack((labels, np.zeros((points_scan_i.shape[0], 1))))
            else:  # append the arrays with laser end-point
                points = np.vstack((points, np.vstack((points_scan_i, laser_endpoint))))
                labels = np.vstack((labels, np.vstack((np.zeros((points_scan_i.shape[0], 1)), np.array([1])[:, np.newaxis]))))

    #pl.scatter(points[:,0], points[:,1], c=labels, s=10)
    #pl.axis('equal')
    #pl.show()
    #sys.exit()
    return np.hstack((points, labels))

def load_obstacles_config(environment):
    """
    :param environment: name of the yaml config file
    :return: all obstacles, area of the environment
    """
    with open(environment+'.yaml') as file:
        yaml_data = yaml.load(file, Loader=yaml.FullLoader)

        # load environment area parameters
        area = yaml_data['area']
        area = (area['x_min'], area['x_max'], area['y_min'], area['y_max'])

        # load static and dynamic obstacles
        obs = yaml_data['obstacles']
        all_obstacles = []
        for i in range(len(obs)):
            obs_i = Obstacle(centroid=[obs[i]['centroid_x'], obs[i]['centroid_y']], dx=obs[i]['dx'], dy=obs[i]['dy'],
                            angle=obs[i]['orientation']*np.pi/180, vel=[obs[i]['velocity_x'], obs[i]['velocity_y']],
                            acc=[obs[i]['acc_x'], obs[i]['acc_y']])
            all_obstacles.append(obs_i)
    return all_obstacles, area

def update_text_file(text_file, data, file_format='carmen'):
    """
    :param text_file: file created - open(output_file_name, 'w')
    :param data: dist_theta for carmen; (T,X,Y) numpy array for txy comma-seperated format
    :param file_format: 'carmen' or 'txy'
    :return:
    """
    if file_format == 'carmen':
        #http://www2.informatik.uni-freiburg.de/~stachnis/datasets.html
        data = ''.join(['%f ' % num for num in data])
        data = 'FLASER 180 ' + data[:-1] + '\n'#Get rid of the last comma
    elif file_format == 'txy':
        data = ''.join(['%f, %f, %f \n' % (t,x,y) for (t,x,y) in data])
    elif file_format == 'txyout':
        data = ''.join(['%f, %f, %f, %f\n' % (t, x, y, out) for (t, x, y, out) in data])
    else:
        pass
    text_file.write(data)




class Sim():

    """
    :param env: name of the yaml file inside the config folder
    :param out_fn: name of the output folder - create this folder inside the output folder
    :param out_file_type: 'txyocc' or 'carmen'
    :param save_all_data_as_npz: True or False
    :param n_reflections: number of lidar beams in the 2D plane
    :param fov: lidar field of view in radiant
    :param max_laser_distance: maximum lidar distance in meters
    :papram unoccupied_points_per_meter: density of zeros between the robot and laser hit
    """

    def __init__(self, env='world1', n_reflections = 360, fov = 3.1415*2, max_laser_distance = 20, unoccupied_points_per_meter = 0.5):
        self.all_obstacles, area = load_obstacles_config(environment=env)
        self.fov = fov
        self.max_laser_distance = max_laser_distance
        self.n_reflections = n_reflections
        self.robot_pose = np.array([0.0, 0.0, 0.0]) # x, y, delta
        self.ranges = []
        self.angles = []
        self.robot_linear_speed = 0.0        
        self.robot_angular_speed = 0.0
        self.lastRunTime = time.time()
        
        
    def setSpeed(self, linearSpeed, angularSpeed):
        self.robot_linear_speed = linearSpeed
        self.robot_angular_speed = angularSpeed

        x = self.robot_linear_speed * 1000
        th = self.robot_angular_speed * (BASE_WIDTH/2) 
        k = max(abs(x-th),abs(x+th))
        # sending commands higher than max speed will fail
        if k > MAX_SPEED:
            x = x*MAX_SPEED/k; th = th*MAX_SPEED/k
        leftSpeed = int(x-th) 
        rightSpeed = int(x+th)
        


    # angles in radiant, ranges in meter
    def getScanRanges(self):
        return self.angles, self.ranges 


    def run(self):
        # update robot
        while True: 
            dT = min(0.5, time.time() - self.lastRunTime) 
            if dT >= 0.2: break
            time.sleep(0.005)
        self.lastRunTime = time.time()                        
        self.robot_pose[0] += self.robot_linear_speed * np.cos(self.robot_pose[2]) * dT   # x
        self.robot_pose[1] += self.robot_linear_speed * np.sin(self.robot_pose[2]) * dT   # y
        self.robot_pose[2] += self.robot_angular_speed * dT                        # delta
        #print('robot_pose', self.robot_pose)

        #update obstacles
        all_obstacle_segments = []
        for obs_i in self.all_obstacles:
            all_obstacle_segments += obs_i.update()

        # update laser reflections
        self.ranges = get_laser_ref(all_obstacle_segments, self.fov, self.n_reflections, self.max_laser_distance, self.robot_pose)
        #print('l1', len(self.ranges))
        self.ranges = list(self.ranges[180:]) + list(self.ranges[:180])  # rotate 
        #print('l2', len(self.ranges))
        self.angles = list(range(0, 360))    
        #print('angles', angles)
        self.angles = [angle / 180.0 * 3.1415 for angle in self.angles]                            
    
    
    def exit(self): 
        pass    
    
    