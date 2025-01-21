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
            #print("false")
            return False
            
#Range-Limited Visbility-Based Voronoi Partitioning 
class RLVBVP2:
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
        
    def get_lidar_data(self, readings):
        self.readings = readings
        return
    
    def read_neighbors(self,neighbours):
        neighbour_coords = []
        neighbour_visPolys = []
        #print(neighbours)
        if neighbours[0] == []:
            return
        for i in neighbours:
            #print(np.linalg.norm(i.pos,self.pos))
            if shapely.distance(Point(i.pos[0],i.pos[1]),Point(self.pos[0],self.pos[1]))<self.comms_radius:
                neighbour_coords.append(i.pos)
                neighbour_visPolys.append(i.visPoly)
        self.neighbour_coords = neighbour_coords
        self.neighbour_visPolys = neighbour_visPolys
        return

    def process_lidar_data(self):
        freePointIdx = []
        occlusionArcs = []
        obstacleLines = []
        temp = []
        for i in range(len(self.readings)):
            if abs(self.readings[i] - self.readings[i-1]) > self.reading_radius * 0.05: #big enough jump for occlusion
                # logger.info(f'adding point {i}')
                # logger.info(f'because {readings[i]}-{readings[i-1]}')
                p1 = Point(0.999*self.readings[i-1]*np.cos(self.angles[i-1])+self.pos[0],0.999*self.readings[i-1]*np.sin(self.angles[i-1])+self.pos[1])
                p2 = Point(0.999*self.readings[i]*np.cos(self.angles[i])+self.pos[0],0.999*self.readings[i]*np.sin(self.angles[i])+self.pos[1])

                occlusionArcs.append(OcclusionArc(self.pos,p1,p2))
                if temp:
                    obstacleLines.append(temp)
                    temp = []

            if self.readings[i] >= self.reading_radius * 0.98: #no collision
                freePointIdx.append(i)
                if temp:
                    obstacleLines.append(temp)
                    temp = []
            else: #collision
                obsPoint = Point(0.999*self.readings[i]*np.cos(self.angles[i])+self.pos[0],\
                                 0.999*self.readings[i]*np.sin(self.angles[i])+self.pos[1])
                temp.append(obsPoint)
            
        if temp:
            obstacleLines.append(temp)
        self.freePointIdx = freePointIdx
        self.occlusionArcs = occlusionArcs
        obstacleLinesObj  = []
        for i in obstacleLines:
            lineObj = LineString(i)
            obstacleLinesObj.append(lineObj)
        self.obstacleLines = obstacleLinesObj
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

    ################################################################################################
    def visibilityPartitioning(self):
        #raycast every point in self.reading_coords
        #TODO: do this for multiple neighbours
        neighbourPoints = []
        count = 0
        vis_count = 0
        for i in range(len(self.reading_coords)):
            point = Point(self.reading_coords[i][0], self.reading_coords[i][1])
            dist_to_self = Point(self.pos[0], self.pos[1]).distance(point)
            dist_to_neighbor = Point(self.neighbour_coords[0][0], self.neighbour_coords[0][1]).distance(point)
            if dist_to_neighbor < dist_to_self:
                count +=1
                # Point is closer to neighbor
                # Create a line from point to neighbor
                neighbor_point = Point(self.neighbour_coords[0][0], self.neighbour_coords[0][1])
                pointLine = Point(point.x*0.999,point.y*0.999)
                line = LineString([pointLine, neighbor_point])
                
                # Check if line intersects with visibility polygon
                clear = True
                for j in self.obstacleLines:
                    if line.intersects(j):
                        clear = False
                        break
                if clear:
                    neighbourPoints.append(point)
                    vis_count +=1
        self.neighbourPoints = neighbourPoints
        self.neighbour_visPolys[0] = Polygon(neighbourPoints)
        print(f'count: {count} vis_count: {vis_count}')
        return

    def slice_visPoly(self):
        self.slicedVisPoly = self.visPoly
        self.slicedVisPoly = self.slicedVisPoly.difference(self.neighbour_visPolys[0])
        return 
    
    def filter_coord_points(self):
        filteredFreePointCoords = []
        for i in self.freePointIdx:
            pointLocation = Point(0.9999*self.readings[i]*np.cos(self.angles[i])+self.pos[0],0.9999*self.readings[i]*np.sin(self.angles[i])+self.pos[1])        
            if not self.neighbour_visPolys[0].contains(pointLocation):
                filteredFreePointCoords.append(pointLocation)

        #TODO: Filter occlusion with raycasting
        filteredOcclusionArcs = []
        #print(f'occ vector points {self.occlusionArcs}')
        for i in self.occlusionArcs:
            if i.checkTruncate(self.slicedVisPoly):
                filteredOcclusionArcs.append(i)
        #print(f'filtered occ arcs {self.filteredOcclusionArcs}')
        self.filteredFreePointCoords = filteredFreePointCoords
        self.filteredOcclusionArcs = filteredOcclusionArcs
        return 
    
    def control_law(self):
        freeArcsComponentArr = np.array([[p.x,p.y]for p in self.filteredFreePointCoords])
        self.freeArcsComponent = sum(freeArcsComponentArr)
        

        return 

    