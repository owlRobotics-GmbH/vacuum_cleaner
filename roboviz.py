'''
roboviz.py - Python classes for displaying maps and robots

Requires: numpy, matplotlib

Copyright (C) 2018 Simon D. Levy

This file is part of PyRoboViz.

PyRoboViz is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

PyRoboViz is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

'''
# Essential imports
import matplotlib.pyplot as plt
import matplotlib.cm as colormap
import matplotlib.lines as mlines
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import cv2

# This helps with Raspberry Pi
import matplotlib
matplotlib.use('TkAgg')
#matplotlib.use('GTK3Agg')
#matplotlib.use('Qt5Agg')


class Visualizer(object):

    # Robot display params
    ROBOT_HEIGHT_M = 0.5
    ROBOT_WIDTH_M  = 0.3

    def __init__(self, map_size_pixels, map_size_meters, title, show_trajectory=False, zero_angle=0, save_figures = False):

        # Put origin in center
        self._init(map_size_pixels, map_size_meters, title, -map_size_pixels / 2, show_trajectory, zero_angle, save_figures)

    def display(self, x_m, y_m, theta_deg):

        self._setPose(x_m, y_m, theta_deg)

        return self._refresh()

    def _init(self, map_size_pixels, map_size_meters, title, shift, show_trajectory=False, zero_angle=0, save_figures = False):

        self.save_figures = save_figures
        if self.save_figures:
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.video=cv2.VideoWriter('out/cleaner.avi',fourcc,30,(500,500))


        # Store constants for update
        map_size_meters = map_size_meters
        self.map_size_pixels = map_size_pixels
        self.map_scale_meters_per_pixel = map_size_meters / float(map_size_pixels)

        # Create a byte array to display the map with a color overlay
        self.bgrbytes = bytearray(map_size_pixels * map_size_pixels * 3)
        
        # Make a nice big (10"x10") figure
        self.fig = plt.figure(figsize=(10,10))

        # Store Python ID of figure to detect window close
        self.figid = id(self.fig)

        self.fig.suptitle('SLAM')        
        #fig.canvas.set_window_title('SLAM')
        plt.title(title)

        # Use an "artist" to speed up map drawing
        self.img_artist = None
        self.img_artist2 = None        
        self.img_artist3 = None        

        # No vehicle to show yet
        self.vehicle = None

        # Create axes
        self.ax = self.fig.gca()
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.grid(False)

        # Hence we must relabel the axis ticks to show millimeters
        ticks = np.arange(shift,self.map_size_pixels+shift+100,100)
        labels = [str(self.map_scale_meters_per_pixel * tick) for tick in ticks]
        self.ax.set_xticklabels(labels)
        self.ax.set_yticklabels(labels)

        # Store previous position for trajectory
        self.prevpos = None
        self.showtraj = show_trajectory

        # We base the axis on pixels, to support displaying the map
        self.ax.set_xlim([shift, self.map_size_pixels+shift])
        self.ax.set_ylim([shift, self.map_size_pixels+shift])

        # Set up default shift for centering at origin
        shift = -self.map_size_pixels / 2

        self.zero_angle = zero_angle
        self.start_angle = None
        self.rotate_angle = 0

        

    def _setPose(self, x_m, y_m, theta_deg):
        '''
        Sets vehicle pose:
        X:      left/right   (m)
        Y:      forward/back (m)
        theta:  rotation (degrees)
        '''

        # If zero-angle was indicated, grab first angle to compute rotation
        if self.start_angle is None and self.zero_angle != 0: 
            self.start_angle = theta_deg
            self.rotate_angle = self.zero_angle - self.start_angle

        # Rotate by computed angle, or zero if no zero-angle indicated
        d = self.rotate_angle
        a = np.radians(d)
        c = np.cos(a)
        s = np.sin(a)
        x_m,y_m = x_m*c-y_m*s, y_m*c+x_m*s

        # Erase previous vehicle image after first iteration
        if not self.vehicle is None:
            self.vehicle.remove()

        # Use a very short arrow shaft to orient the head of the arrow
        theta_rad = np.radians(theta_deg+d)
        c = np.cos(theta_rad)
        s = np.sin(theta_rad)
        l = 0.1
        dx = l * c
        dy = l * s
 
        s = self.map_scale_meters_per_pixel

        self.vehicle=self.ax.arrow(x_m/s, y_m/s, 
                dx, dy, head_width=Visualizer.ROBOT_WIDTH_M/s, 
                head_length=Visualizer.ROBOT_HEIGHT_M/s, fc='r', ec='r', alpha=0.5)

        # Show trajectory if indicated
        currpos = self._m2pix(x_m,y_m)
        if self.showtraj and not self.prevpos is None:
            self.ax.add_line(mlines.Line2D((self.prevpos[0],currpos[0]), (self.prevpos[1],currpos[1]), alpha=1.0))
        self.prevpos = currpos

    def _refresh(self):                   

        # If we have a new figure, something went wrong (closing figure failed)
        if self.figid != id(plt.gcf()):
            return False

        # Redraw current objects without blocking
        plt.draw()

        # Refresh display, setting flag on window close or keyboard interrupt
        try:
            plt.pause(.001) # Arbitrary pause to force redraw
            if self.save_figures:
                plt.savefig('out/fig.png')
                img = cv2.imread('out/fig.png')
                img = cv2.resize(img, (500,500), interpolation = cv2.INTER_NEAREST)             
                print(img.shape)
                #self.fig.canvas.draw()
                #img = np.array(plt.canvas.renderer.buffer_rgba())
                #img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                #cv2.imshow('img', img)
                self.video.write(img)
            return True
        except:
            print('refresh exception')
            return False

        return True

    def _m2pix(self, x_m, y_m):

        s = self.map_scale_meters_per_pixel

        return x_m/s, y_m/s
    
class MapVisualizer(Visualizer):
    
    def __init__(self, map_size_pixels, map_size_meters, title='MapVisualizer', show_trajectory=False, save_figures=False):

        # Put origin in lower left; disallow zero-angle setting
        Visualizer._init(self, map_size_pixels, map_size_meters, title, 0, show_trajectory, 0, save_figures)

    def display(self, x_m, y_m, theta_deg, mapimg, cleanimg, routeimg):

        self._setPose(x_m, y_m, theta_deg)

        #mapimg = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8), (self.map_size_pixels, self.map_size_pixels))

        # Pause to allow display to refresh
        plt.pause(.001)

        if self.img_artist is None:

            self.img_artist = self.ax.imshow(mapimg, cmap=colormap.gray, alpha=0.6)
            self.img_artist2 = self.ax.imshow(cleanimg, cmap=colormap.plasma, alpha=0.4)             
            self.img_artist3 = self.ax.imshow(routeimg, cmap=colormap.plasma, alpha=0.4)             

        else:

            self.img_artist.set_data(mapimg)
            self.img_artist2.set_data(cleanimg)
            self.img_artist3.set_data(routeimg)

        return self._refresh()

