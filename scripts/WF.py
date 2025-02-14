import math
import shapely
from shapely import MultiPoint
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union,split
from shapely.geometry import LineString
import logging
import matplotlib.pyplot as plt
import numpy as np
import os             
            
#Wall Following
class WF:
    def __init__(self, pos,reading_radius,resolution,id):
        self.pos = pos
        self.theta=  0.0
        self.id = id
        self.reading_radius = reading_radius
        self.resolution = resolution
        self.readings = []
        self.angles = np.linspace(0.0,-2*np.pi, resolution, endpoint=False)
        self.reading_coords = []

        self.state = 1

        # self.logger = logging.getLogger(__name__)
        # this_dir, this_filename = os.path.split(__file__)
        # self.myfile = os.path.join(this_dir, lidar_path) 

    def transformationMatrix(self,cmd):
        newpos = np.zeros(2)
        newpos[0] = cmd[0] * np.cos(self.theta) - cmd[1] * np.sin(self.theta)
        newpos[1] = cmd[0] * np.sin(self.theta) + cmd[1] * np.cos(self.theta)
        return newpos

    def read_lidar_data(self,path = None): #read from filepath
        if not path:
            filepath = self.myfile
        else:
            filepath = path
        try:
            with open(filepath, 'r') as file:
                # Read lines and convert to float, replacing 'inf' with max_range
                readings = [0.0 for i in range(540)]
                for line in file:
                    temp = line[1:-1].split(", ")
                for i in range(len(temp)):
                    perm = 0.0
                    if temp[i] == 'inf':
                        perm = self.reading_radius
                    else:
                        perm = round(float(temp[i]), 3)
                    readings[i]=perm
            self.readings = readings
            reading_coords = np.array([[self.readings[i]*np.cos(self.angles[i])+self.pos[0],
                                 self.readings[i]*np.sin(self.angles[i])+self.pos[1]] 
                                 for i in range(len(self.readings))])
            self.reading_coords = reading_coords
            return

        except FileNotFoundError:
            self.logger.info("Error: lidar_reading.txt not found")
            return
        
    def get_lidar_data(self, readings): #read from direct array
        readings_temp = [0.0 for i in range(self.resolution)]
        for i in range(len(readings)):
            if readings[i] == float('inf'):        
                readings_temp[i] = self.reading_radius
            else:
                readings_temp[i] = readings[i]
        self.readings = readings_temp
        # print(f'readings: {len(self.readings)}')
        # print(f'angles: {len(self.angles)}')
        reading_coords = np.array([[self.readings[i]*np.cos(self.angles[i])+self.pos[0],
                                 self.readings[i]*np.sin(self.angles[i])+self.pos[1]] 
                                 for i in range(len(self.readings))])
        self.reading_coords = reading_coords
        return

    def process_lidar_data(self):
        print(f'right_readings start = {self.reading_coords[-2:]}')
        right_readings = np.concatenate((self.reading_coords[-2:], self.reading_coords[:2]))
        front_readings = self.reading_coords[(int(self.resolution * 3/4))-1:(int(self.resolution * 3/4))+1]
        print(f'right readings: {right_readings}')
        print(f'front_readings: {front_readings}')
        # Check if all readings are below threshold
        threshold = 1.0  # Assuming a threshold of 1.0 meters
        right_collision = all(np.linalg.norm(reading - self.pos) < threshold for reading in right_readings)
        front_collision = all(np.linalg.norm(reading - self.pos) < threshold for reading in front_readings)
        x = [np.linalg.norm(reading - self.pos) for reading in right_readings]
        y = [np.linalg.norm(reading - self.pos) for reading in front_readings]
        print(f'right_distances: {x}')
        print(f'front_distances: {y}')
        print(f'my pos: {self.pos}')
        print(f'right_collison: {right_collision}')
        print(f'front_collison: {front_collision}')
        if self.state != 0:
            angle_error = 0.0
            first_front_point = front_readings[0]
            last_front_point = front_readings[-1]
            front_angle = np.arctan2(last_front_point[1] - first_front_point[1], 
                                last_front_point[0] - first_front_point[0])
            first_right_point = right_readings[0]
            last__right_point = right_readings[-1]
            right_angle = np.arctan2(last__right_point[1] - first_right_point[1], 
                                last__right_point[0] - first_right_point[0])

            if right_collision and front_collision: #turn left
                angle_error = front_angle - self.theta 
            elif right_collision and not front_collision: #follow right
                angle_error = right_angle - self.theta 
            elif not right_collision and front_collision: #turn left
                angle_error = front_angle - self.theta 
            elif not right_collision and not front_collision: #turn right
                angle_error = 0.1
            self.theta += angle_error * 0.1 *32/1000
            print(f'angle error: {angle_error}')
        return

    def move(self,timestep,log = False,move = True):
        return
    def step(self,range_image,timestep,log=False,move=True):
        self.get_lidar_data(range_image)
        self.process_lidar_data() #find self.freePointIdx and self.occlusionArcs
        #self.move(timestep,log,move)