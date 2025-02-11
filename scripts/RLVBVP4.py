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
        else:
            self.interpolated_points = []
        self.filtered_points = []
        self.filteredLineString = None                
            
#Range-Limited Visbility-Based Voronoi Partitioning 
class RLVBVP3:
    def __init__(self, pos, comms_radius,reading_radius,resolution,lidar_path = 'lidar_reading2.txt',id=0):
        self.pos = pos
        self.id = id
        self.comms_radius = comms_radius
        self.reading_radius = reading_radius
        self.resolution = resolution
        self.readings = []
        self.angles = np.linspace(0.5*np.pi, -1.5*np.pi, resolution, endpoint=False)
        self.reading_coords = []

        self.freePointCoords = [] #list of 2d arr
        self.occlusionArcs = [] 

        self.neighbour_coords = []

        self.filteredFreePointCoords=[] #list of Points
        #self.filteredOcclusionCoords=[]

        self.freeArcsComponent = [0.0,0.0]
        self.occlusionArcsComponent = [0.0,0.0]
        
        self.logger = logging.getLogger(__name__)
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
        freePointCoord = []
        occlusionCoord = []
        obstacleLines = []
        temp = []
        #print(len(self.readings))
        #print(self.readings)
        #print(self.angles)
        for i in range(len(self.readings)):
            
            if abs(self.readings[i] - self.readings[i-1]) > self.reading_radius * 0.05: #big enough jump for occlusion
                # logger.info(f'adding point {i}')
                # logger.info(f'because {readings[i]}-{readings[i-1]}')
                p1 = Point(0.999*self.readings[i-1]*np.cos(self.angles[i-1])+self.pos[0],0.999*self.readings[i-1]*np.sin(self.angles[i-1])+self.pos[1])
                p2 = Point(0.999*self.readings[i]*np.cos(self.angles[i])+self.pos[0],0.999*self.readings[i]*np.sin(self.angles[i])+self.pos[1])

                try:
                    occlusionCoord.append(OcclusionArc(self.pos,p1,p2))
                except ValueError:
                    print("Value error")
                    print(self.pos)
                    print(p1.xy)
                    print(p2.xy)
                    raise Exception
                if temp:
                    obstacleLines.append(temp)
                    temp = []

            if self.readings[i] >= self.reading_radius * 0.99: #no collision
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
            if len(i)>1:
                lineObj = LineString(i)
                obstacleLinesObj.append(lineObj)
        self.obstacleLines = obstacleLinesObj
        return 

    ################################################################################################
    def visibilityPartitioning(self):
        #raycast every point in self.reading_coords
        #TODO: do this for multiple neighbours i.e. hardcoded self.neighbour_coords[0]
        #TODO: relative point positioning
        filteredFreePointCoords = []
        #filteredOcclusionCoords = []

        if not self.neighbour_coords: # no neighbours
            filteredFreePointCoords = [Point(i[0], i[1]) for i in self.freePointCoords]
            self.filteredFreePointCoords = filteredFreePointCoords
            for i in self.occlusionArcs:
                for j in i.interpolated_points:
                    point = Point(j[0], j[1])
                    i.filtered_points.append(point)
                #filteredOcclusionCoords.append(point)
                if len(i.filtered_points)>2:
                    i.filteredLineString = LineString(i.filtered_points)
            #self.filteredOcclusionCoords = filteredOcclusionCoords
            return 
        
        for i in self.freePointCoords:
            point = Point(i[0], i[1])
            dist_to_self = Point(self.pos[0], self.pos[1]).distance(point)
            neighbourCanSee = False
            for j in self.neighbour_coords:
                neighbourPoint = Point(j[0], j[1])
                dist_to_neighbor = neighbourPoint.distance(point)
                if dist_to_neighbor < dist_to_self:
                    line = LineString([point,neighbourPoint])
                    clear = True
                    for k in self.obstacleLines: #and occlusion lines?
                        if line.intersects(k):
                            clear = False 
                            break #this free point cannot be seen by this neighbour
                    if clear:
                        neighbourCanSee = True
                        break
            if not neighbourCanSee:
                filteredFreePointCoords.append(point)

        self.filteredFreePointCoords = filteredFreePointCoords[:]

        
        for i in self.occlusionArcs:
            for j in i.interpolated_points:
                point = Point(j[0], j[1])
                dist_to_self = Point(self.pos[0], self.pos[1]).distance(point)
                neighbourCanCover = False
                for k in self.neighbour_coords:
                    neighbourPoint = Point(k[0], k[1])
                    dist_to_neighbor = neighbourPoint.distance(point)
                    line = LineString([point,neighbourPoint])
                    clear = True
                    for l in self.obstacleLines: #and occlusion lines?
                        if line.intersects(l):
                            clear = False
                            break
                    within_range = True if dist_to_neighbor < self.reading_radius else False
                    if dist_to_neighbor < dist_to_self:
                        if clear:
                            neighbourCanCover = True
                    else:
                        if clear and within_range:
                            neighbourCanCover = True
                if not neighbourCanCover:
                    i.filtered_points.append(point)
            if len(i.filtered_points)>2:
                i.filteredLineString = LineString(i.filtered_points)
        return

    def control_law(self):
        freeArcsComponentArr = np.zeros(2)
        for i in self.filteredFreePointCoords:
            # Calculate direction vector from robot to point
            direction = np.array([i.x - self.pos[0], i.y - self.pos[1]])
            # Normalize to unit vector
            norm = np.linalg.norm(direction)
            if norm > 0:
                unit_vector = direction / norm
                freeArcsComponentArr += unit_vector
        #print(f'free length: {len(self.filteredFreePointCoords)}')
        self.freeArcsComponent = freeArcsComponentArr

        occ_length = 0
        readingPerUnitLength = self.resolution/(2 * np.pi * self.reading_radius)
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
                #print(f'nearest: {nearest_idx}, furthest: {furthest_idx}')
                
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
                
                densityComponent = (1-math.pow(np.linalg.norm(pa-pr)/np.linalg.norm(pb-pr),2))/2
                #print(f'density component: {densityComponent}')
                tempComponent = normal * math.pow(np.linalg.norm(pb-pr),2) / np.linalg.norm(pr-self.pos) * densityComponent
                #print(f'temp component: {normal} * {math.pow(np.linalg.norm(pb-pa),2)} / {(np.linalg.norm(pr-self.pos) * densityComponent)}')
                #print(f'adding temp occ component: {tempComponent}')
                occlusionArcsComponentArr += tempComponent * readingPerUnitLength
        #print(f'occ length: {occ_length}')
        self.occlusionArcsComponent = occlusionArcsComponentArr
        return occ_length
    

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