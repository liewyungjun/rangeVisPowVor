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
            
#Range-Limited Visbility-Based Voronoi Partitioning 
class RLVBVP_F2:
    def __init__(self, pos, comms_radius,reading_radius,resolution,
                 lidar_path = 'lidar_reading2.txt',id=0,fov=60,ori=0.0):
        self.pos = pos
        self.ori = ori
        self.id = id
        self.comms_radius = comms_radius
        self.reading_radius = reading_radius
        self.resolution = resolution
        self.readings = []
        angle_start = 0.5*np.pi + ((fov/2)/360)*2*np.pi
        angle_end = 0.5*np.pi - ((fov/2)/360)*2*np.pi
        self.angles = np.linspace(angle_start, angle_end, resolution)
        self.reading_coords = []

        self.fov_in_radians = fov/360 * 2 * np.pi
        self.fov = fov
        self.neighbour_coords = []
        
        self.visPoly = None
        self.readingPoly = None
        self.avoidPolys = []

        this_dir, this_filename = os.path.split(__file__)
        self.myfile = os.path.join(this_dir, lidar_path) 

    def read_lidar_data(self,path = None): #read from filepath
        if not path:
            filepath = self.myfile
        else:
            filepath = path
        try:
            with open(filepath, 'r') as file:
                # Read lines and convert to float, replacing 'inf' with max_range
                readings = [0.0 for i in range(self.resolution)]
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
        reading_coords = np.array([[self.readings[i]*np.cos(self.angles[i])+self.pos[0],
                                 self.readings[i]*np.sin(self.angles[i])+self.pos[1]] 
                                 for i in range(len(self.readings))])
        self.reading_coords = reading_coords
        #print(f"read in {len(readings)}")
        #print(self.reading_coords)
        return
    
    def read_neighbors(self,neighbours):
        neighbour_coords = []
        if neighbours[0] == []:
            return
        for i in neighbours:
            if shapely.distance(Point(i[0],i[1]),Point(self.pos[0],self.pos[1]))<self.comms_radius:
                neighbour_coords.append(i)
        self.neighbour_coords = neighbour_coords
        return

    def process_lidar_data(self):
        self.avoidPolys = []
        visPolyRange = np.array([[self.reading_radius * np.cos(angle)  + self.pos[0],
                                    self.reading_radius * np.sin(angle) + self.pos[1]]
                                    for angle in self.angles])
        self.readingPoly = Polygon(np.vstack((self.reading_coords,self.pos)))
        self.visPoly = Polygon(np.vstack((visPolyRange,self.pos)))
        #TODO: find intersections
        difference = shapely.difference(self.visPoly,self.readingPoly)
        if not difference.is_empty:
            self.avoidPolys.append(difference)
        return 

    ################################################################################################
    def visibilityPartitioning(self):
        #partition with inferred neighbour cone
        if self.neighbour_coords:
            # Calculate midpoint
            neighbor_pos = Point(self.neighbour_coords[0])
            midpoint_x = (self.pos[0] + neighbor_pos.x) / 2
            midpoint_y = (self.pos[1] + neighbor_pos.y) / 2

            # Calculate perpendicular slope (negative reciprocal)
            dx = neighbor_pos.x - self.pos[0]
            dy = neighbor_pos.y - self.pos[1]
            perp_dx = -dy
            perp_dy = dx

            # Normalize perpendicular vector and scale it
            scale = 1.0  # Length of perpendicular line
            length = np.sqrt(perp_dx**2 + perp_dy**2)
            perp_dx = (perp_dx / length) * scale
            perp_dy = (perp_dy / length) * scale

            # Create a line that extends far beyond the intersection
            line_length = 100  # Make it long enough to cross the entire intersection
            bisector_line = LineString([
                (midpoint_x - perp_dx * line_length, midpoint_y - perp_dy * line_length),
                (midpoint_x + perp_dx * line_length, midpoint_y + perp_dy * line_length)
            ])

            
            # Create neighbor's visibility polygon as a circle
            neighbor_poly = neighbor_pos.buffer(self.reading_radius)
            
            
            # Find intersection between visibility polygons
            intersection = self.readingPoly.intersection(neighbor_poly)
            
            if not intersection.is_empty:
                # Split intersection by bisector
                split_parts = split(intersection, bisector_line)
                
                # Determine which part is further from self.pos
                my_point = Point(self.pos)
                for i in split_parts.geoms:
                    if i.centroid.distance(my_point) > i.centroid.distance(neighbor_pos):
                        #TODO: there is a bug where one point remains
                        self.readingPoly = self.readingPoly.difference(i)
                        self.avoidPolys.append(i)
        return

    def control_law(self):
        #control law based on overlap area and centroids
        component = np.zeros(2)
        for i in self.avoidPolys:
            avoidVector = np.array([self.pos[0] - i.centroid.x,self.pos[1] - i.centroid.y])
            component +=avoidVector * i.area
        return component
    

    def move(self,timestep,log = False):
        self.velocity =self.freeArcsComponent + self.occlusionArcsComponent
        movement_step = self.velocity * timestep/25000
        movement_step = np.clip(movement_step, -5.0*32/1000, 5.0*32/1000)
        
        #self.readPos()
        self.pos +=movement_step
        if log:
            self.writeReadings()
    
    def readPos(self,positionReading):
        if positionReading:
            self.pos[0] = positionReading[0]
            self.pos[1] = positionReading[1]

    def writeReadings(self):
        with open(f'webots{self.id}.txt', 'w') as f:
                    f.truncate(0)
                    f.write('[')
                    for reading in self.readings[:-1]:
                        f.write(f"{reading}, ")
                    f.write(f"{self.readings[-1]}")   
                    f.write(']')
        with open(f'webots_coords_{self.id}.tmp', 'w') as f:
                    f.truncate(0)
                    f.write('[')
                    for reading in self.reading_coords[:-1]:
                        f.write(f"{reading}, ")
                    f.write(f"{self.reading_coords[-1]}")   
                    f.write(']')
        os.replace(f'webots_coords_{self.id}.tmp',f'webots_coords_{self.id}.txt')