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
        # Calculate linear distance between consecutive points
        point1_coords = np.array([point1.x, point1.y])
        point2_coords = np.array([point2.x, point2.y])
        self.linear_distance = np.linalg.norm(point2_coords - point1_coords)
        
        # Interpolate points along the arc with consistent spacing
        spacing = 2 * np.pi * 3.0 / 540
        num_points = int(self.linear_distance / spacing)  # 0.1m spacing
        if num_points > 1:
            t = np.linspace(0, 1, num_points)
            self.interpolated_points = np.array([(1-t_)*point1_coords + t_*point2_coords for t_ in t])
            #self.interpolated_arc = LineString(interpolated_points)
        self.filtered_points = []
        self.filteredLineString = None
        
        
            
#Range-Limited Visbility-Based Voronoi Partitioning 
class RLVBVP3:
    def __init__(self, pos, comms_radius,reading_radius,resolution,lidar_path = 'lidar_reading2.txt'):
        self.pos = pos
        self.comms_radius = comms_radius
        self.reading_radius = reading_radius
        self.readings = []
        self.angles = np.linspace(0.5*np.pi, -1.5*np.pi, resolution)
        self.reading_coords = []

        self.freePointCoords = [] #list of 2d arr
        self.occlusionArcs = [] 

        self.neighbour_coords = [[2.2,0.0]]

        self.filteredFreePointCoords=[] #list of Points
        self.filteredOcclusionCoords=[]

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
            reading_coords = np.array([[self.readings[i]*np.cos(self.angles[i])+self.pos[0],
                                 self.pos[1]+self.readings[i]*np.sin(self.angles[i])] 
                                 for i in range(len(self.readings))])
            self.reading_coords = reading_coords
            return

        except FileNotFoundError:
            self.logger.info("Error: lidar_reading.txt not found")
            return
        
    def get_lidar_data(self, readings):
        self.readings = readings
        return
    
    def read_neighbors(self,neighbours):
        neighbour_coords = []
        if neighbours[0] == []:
            return
        for i in neighbours:
            if shapely.distance(Point(i.pos[0],i.pos[1]),Point(self.pos[0],self.pos[1]))<self.comms_radius:
                neighbour_coords.append(i.pos)
        self.neighbour_coords = neighbour_coords
        return

    def process_lidar_data(self):
        freePointCoord = []
        occlusionCoord = []
        obstacleLines = []
        temp = []
        for i in range(len(self.readings)):
            if abs(self.readings[i] - self.readings[i-1]) > self.reading_radius * 0.05: #big enough jump for occlusion
                # logger.info(f'adding point {i}')
                # logger.info(f'because {readings[i]}-{readings[i-1]}')
                p1 = Point(0.999*self.readings[i-1]*np.cos(self.angles[i-1])+self.pos[0],0.999*self.readings[i-1]*np.sin(self.angles[i-1])+self.pos[1])
                p2 = Point(0.999*self.readings[i]*np.cos(self.angles[i])+self.pos[0],0.999*self.readings[i]*np.sin(self.angles[i])+self.pos[1])

                occlusionCoord.append(OcclusionArc(self.pos,p1,p2))
                if temp:
                    obstacleLines.append(temp)
                    temp = []

            if self.readings[i] >= self.reading_radius * 0.98: #no collision
                freePointCoord.append(self.reading_coords[i])
                if temp:
                    obstacleLines.append(temp)
                    temp = []
            else: #collision
                obsPoint = Point(0.999*self.readings[i]*np.cos(self.angles[i])+self.pos[0],\
                                 0.999*self.readings[i]*np.sin(self.angles[i])+self.pos[1])
                temp.append(obsPoint)
            
        if temp:
            obstacleLines.append(temp)
        self.freePointCoords = freePointCoord
        self.occlusionArcs = occlusionCoord
        obstacleLinesObj  = []
        for i in obstacleLines:
            lineObj = LineString(i)
            obstacleLinesObj.append(lineObj)
        self.obstacleLines = obstacleLinesObj
        return 

    ################################################################################################
    def visibilityPartitioning(self):
        #raycast every point in self.reading_coords
        #TODO: do this for multiple neighbours
        filteredFreePointCoords = []
        for i in self.freePointCoords:
            point = Point(i[0], i[1])
            dist_to_self = Point(self.pos[0], self.pos[1]).distance(point)
            neighbourPoint = Point(self.neighbour_coords[0][0], self.neighbour_coords[0][1])
            dist_to_neighbor = neighbourPoint.distance(point)
            if dist_to_neighbor < dist_to_self:
                line = LineString([point,neighbourPoint])
                clear = True
                for j in self.obstacleLines: #and occlusion lines?
                    if line.intersects(j):
                        clear = False
                        break
                if not clear:
                    filteredFreePointCoords.append(point)
            else:
                filteredFreePointCoords.append(point)
        self.filteredFreePointCoords = filteredFreePointCoords

        filteredOcclusionCoords = []
        for i in self.occlusionArcs:
            for j in i.interpolated_points:
                point = Point(j[0], j[1])
                dist_to_self = Point(self.pos[0], self.pos[1]).distance(point)
                neighbourPoint = Point(self.neighbour_coords[0][0], self.neighbour_coords[0][1])
                dist_to_neighbor = neighbourPoint.distance(point)
                if dist_to_neighbor < dist_to_self:
                    line = LineString([point,neighbourPoint])
                    clear = True
                    for j in self.obstacleLines: #and occlusion lines?
                        if line.intersects(j):
                            clear = False
                            break
                    if not clear:
                        i.filtered_points.append(point)
                        filteredOcclusionCoords.append(point)
                else:
                    i.filtered_points.append(point)
                    filteredOcclusionCoords.append(point)
            if len(i.filtered_points)>2:
                i.filteredLineString = LineString(i.filtered_points)
        self.filteredOcclusionCoords = filteredOcclusionCoords

        return

    def control_law(self):
        freeArcsComponentArr = np.array([[p.x,p.y]for p in self.filteredFreePointCoords])
        print(f'free length: {len(self.filteredFreePointCoords)}')
        self.freeArcsComponent = sum(freeArcsComponentArr)

        occ_length = 0
        occlusionArcsComponentArr = np.array([0.0,0.0])
        for i in self.occlusionArcs:
            occlusionLine = i.filteredLineString
            if len(i.filtered_points) > 2:
                # Get coordinates of all points on the line
                coords = np.array(occlusionLine.coords)
                occ_length += len(coords)
                # Find nearest and furthest points
                distances = [Point(self.pos[0], self.pos[1]).distance(Point(x, y)) for x, y in coords]
                nearest_idx = np.argmin(distances)
                furthest_idx = np.argmax(distances)
                
                pa = coords[nearest_idx]
                pb = coords[furthest_idx]
                pr = np.array([i.reflex.x,i.reflex.y])
                
                # Calculate direction vector of the line
                line_vector = coords[-1] - coords[0]
                
                # Calculate normal vector (rotate 90 degrees counterclockwise)
                normal = np.array([-line_vector[1], line_vector[0]])
                
                # Normalize the normal vector
                normal = normal / np.linalg.norm(normal)
                
                # Ensure normal points inward by checking if it points towards self.pos
                center_to_line = np.array([self.pos[0], self.pos[1]]) - coords[0]
                if np.dot(normal, center_to_line) < 0:
                    normal = -normal
                
                densityComponent = math.pow(1-np.linalg.norm((pa-pr)/(pb-pr)),2)/2

                tempComponent = normal * math.pow(np.linalg.norm(pb-pa),2) / (np.linalg.norm(pr-self.pos) * densityComponent)
                occlusionArcsComponentArr += tempComponent
        print(f'occ length: {occ_length}')
        self.occlusionArcsComponent = occlusionArcsComponentArr
        return 

    