import math
from shapely import MultiPoint
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union,split
from shapely.geometry import LineString
import logging
import matplotlib.pyplot as plt
import numpy as np
import os

class OcclusionArc:
    def __init__(self,pos,point1,point2):
        self.point1 = point1
        self.point2 = point2
        self.reflex =  point1 if np.linalg.norm(np.array([point1.x, point1.y]) - np.array(pos)) \
                        < np.linalg.norm(np.array([point2.x, point2.y]) - np.array(pos)) else point2        
        self.occlusionArc = LineString([point1,point2])
        self.length = self.occlusionArc.length
        
    def checkTruncate(self,slicedVisPoly):
        point1_in = slicedVisPoly.contains(self.point1)
        point2_in = slicedVisPoly.contains(self.point2)
        
        if point1_in and point2_in:
            # Both points inside, add as is
            return True
        elif point1_in or point2_in:
            # One point inside, one outside - find intersection with boundary
            intersection = self.occlusionArc.intersection(slicedVisPoly.boundary)
            if not intersection.is_empty:
                if isinstance(intersection, Point):
                    # Single intersection point
                    if point1_in:
                        self.point2 = intersection
                    else:
                        self.point1 = intersection
                elif isinstance(intersection, MultiPoint):
                    # Multiple intersection points - take closest to inside point
                    if point1_in:
                        closest_point = min(intersection.geoms, key=lambda p: self.point1.distance(p))
                        self.point2 = closest_point
                    else:
                        closest_point = min(intersection.geoms, key=lambda p: self.point2.distance(p))
                        self.point1 = closest_point
            return True
        else:
            print("false")
            return False
            
#Range-Limited Visbility-Based Voronoi Partitioning 
class RLVBVP:
    def __init__(self, pos, comms_radius,reading_radius,resolution,lidar_path = 'lidar_reading2.txt'):
        self.pos = pos
        self.comms_radius = comms_radius
        self.reading_radius = reading_radius
        self.readings = []
        self.angles = np.linspace(0.5*np.pi, -1.5*np.pi, resolution)

        self.freePointIdx = [] #list of indexes for free points
        self.occlusionArcs = [] #list of occlusion arcs objects

        self.reading_coords = []
        self.visPoly = Polygon()

        self.neighbour_coords = [[2.2,0.0]]
        self.neighbour_visPolys = []

        self.intersectionPolys = None
        self.intersectionPolysAllocation = None

        self.slicedVisPoly = None

        self.filteredFreePointCoords=[]
        self.filteredOcclusionArcs=[]
        self.freeArcsComponent = [0.0,0.0]
        self.occlusionArcsComponent = [0.0,0.0]
        
        self.logger = logging.getLogger(__name__)
        this_dir, this_filename = os.path.split(__file__)
        self.myfile = os.path.join(this_dir, lidar_path) 

    #TODO: add inputs to the class: reading neighbour locations
    def read_lidar_data(self):
        try:
            with open(self.myfile, 'r') as file:
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
            return

        except FileNotFoundError:
            self.logger.info("Error: lidar_reading.txt not found")
            return
    
    def read_neighbor(self,neighbour_coord):
        self.neighbour_coords = neighbour_coord
        return

    def process_lidar_data(self):
        freePointIdx = []
        occlusionArcs = []
        for i in range(len(self.readings)):
            if self.readings[i] >= self.reading_radius * 0.98: #no collision
                freePointIdx.append(i)
            if abs(self.readings[i] - self.readings[i-1]) > self.reading_radius * 0.05: #big enough jump for occlusion
                # logger.info(f'adding point {i}')
                # logger.info(f'because {readings[i]}-{readings[i-1]}')
                p1 = Point(0.999*self.readings[i-1]*np.cos(self.angles[i-1])+self.pos[0],0.999*self.readings[i-1]*np.sin(self.angles[i-1])+self.pos[1])
                p2 = Point(0.999*self.readings[i]*np.cos(self.angles[i])+self.pos[0],0.999*self.readings[i]*np.sin(self.angles[i])+self.pos[1])

                occlusionArcs.append(OcclusionArc(self.pos,p1,p2))
        self.freePointIdx = freePointIdx
        self.occlusionArcs = occlusionArcs
        return 
    
    def generate_coordinatesWithPolygon(self):
        poly_coords = np.array([[self.readings[i]*np.cos(self.angles[i])+self.pos[0],
                                 self.pos[1]+self.readings[i]*np.sin(self.angles[i])] 
                                 for i in range(len(self.readings))])
        visPoly = Polygon(poly_coords)
        self.reading_coords = poly_coords
        self.visPoly = visPoly
        return

    def generate_neighbourCoordsWithPolygon(self):
        neighbour_VisPolys = []
        for i in self.neighbour_coords:
            #if np.linalg.norm(self.pos,i) < self.comms_radius:
            circle_coords = np.array([[np.cos(theta)*self.reading_radius+i[0], np.sin(theta)*self.reading_radius+i[1]] for theta in np.linspace(0, 2*np.pi, 100)])
            circle_poly = Polygon(circle_coords)
            neighbour_VisPolys.append(circle_poly)
        self.neighbour_visPolys = neighbour_VisPolys
        return

    def visibilityPartitioning(self):#find intersection polygon and partition it
    #find intersection polygon and midpoint
    #TODO: do this for multiple neighbours
        circle_poly = self.neighbour_visPolys[0]
        intersection = self.visPoly.intersection(circle_poly)
        midpoint_x = (self.pos[0] + self.neighbour_coords[0][0]) / 2
        midpoint_y = (self.pos[1] + self.neighbour_coords[0][1]) / 2

        # Calculate perpendicular vector
        # Original vector is (self.neighbour_coords[0].x - self.pos.x, self.neighbour_coords[0].y - self.pos.y)
        # Perpendicular vector is (-dy, dx)
        dx = self.neighbour_coords[0][0] - self.pos[0]
        dy = self.neighbour_coords[0][1] - self.pos[1]
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

        # Split the intersection polygon with the bisector line
        split_polys = split(intersection, bisector_line)
        split_poly_colors = ['' for p in split_polys.geoms]
        # For each part of the split intersection, determine which starting point it's closer to
        for poly in range(len(split_polys.geoms)):
            # Get centroid of the polygon part
            centroid = split_polys.geoms[poly].centroid
            
            # Calculate distances to starting points
            dist_to_self_pos = Point(self.pos[0], self.pos[1]).distance(centroid)
            dist_to_self_neighbour_coords = Point(self.neighbour_coords[0][0], self.neighbour_coords[0][1]).distance(centroid)
            
            # Color based on closest starting point
            split_poly_colors[poly] = 'lightblue' if dist_to_self_pos < dist_to_self_neighbour_coords else 'lightcoral'
        self.intersectionPolys = split_polys
        self.intersectionPolysAllocation = split_poly_colors
        self.bisector_line = bisector_line
        return 

    def slice_visPoly(self):#TODO: improve with point raycasting
        self.slicedVisPoly = self.visPoly
        for i in range(len(self.intersectionPolysAllocation)):
            if self.intersectionPolysAllocation[i] == 'lightcoral':
                self.slicedVisPoly = self.slicedVisPoly.difference(self.intersectionPolys.geoms[i])
                #print("filtering shape")
        return 
    
    def filter_coord_points(self):
        filteredFreePointCoords = []
        for i in self.freePointIdx:
            pointLocation = Point(0.9999*self.readings[i]*np.cos(self.angles[i])+self.pos[0],0.9999*self.readings[i]*np.sin(self.angles[i])+self.pos[1])        
            if self.slicedVisPoly.contains(pointLocation):
                filteredFreePointCoords.append(pointLocation)

        #TODO: Filter occlusion with raycasting
        filteredOcclusionArcs = []
        print(f'occ vector points {self.occlusionArcs}')
        for i in self.occlusionArcs:
            if i.checkTruncate(self.slicedVisPoly):
                filteredOcclusionArcs.append(i)
        print(f'filtered occ arcs {self.filteredOcclusionArcs}')
        self.filteredFreePointCoords = filteredFreePointCoords
        self.filteredOcclusionArcs = filteredOcclusionArcs
        return 
    
    def control_law(self):
        freeArcsComponentArr = np.array([[p.x,p.y]for p in self.filteredFreePointCoords])
        self.freeArcsComponent = sum(freeArcsComponentArr)
        

        return 

    