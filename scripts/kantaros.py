import math
import matplotlib.pyplot as plt
import numpy as np
import os
from lidar_processing_points import * 
import logging
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union,split
from shapely.geometry import LineString
from kantaros_utils import *
start1= [0.0,0.0]
start2 = [2.2,0]
radius1 = 3.0
radius2 = 3.0

readings = read_lidar_data()
freePointIdx,occlusionVectorPoints = process_lidar_data(readings)
angles = np.linspace(0.5*np.pi, -1.5*np.pi, len(readings))
poly_coords = np.array([[readings[i]*np.cos(angles[i])+start1[0],start1[1]+readings[i]*np.sin(angles[i])] for i in range(len(readings))])
visPoly = Polygon(poly_coords)

#hypothetical neighbour
circle_coords = np.array([[np.cos(theta)*radius2+start2[0], np.sin(theta)*radius2+start2[1]] for theta in np.linspace(0, 2*np.pi, 100)])
circle_poly = Polygon(circle_coords)

#find intersection and allocate based on voronoi
split_polys,split_poly_colors = visibilityPartitioning(visPoly,circle_poly,start1,start2)

plt.figure()
#plot original visPoly and neighbour and intersection division
x, y = visPoly.exterior.xy
plt.plot(x, y, 'b-', label='Visibility Polygon')
x, y = circle_poly.exterior.xy
plt.plot(x, y, 'r-', label='Circle')
plt.scatter(start1[0], start1[1], label='Drone 1')
plt.scatter(start2[0], start2[1], label='Drone 2')
for poly in range(len(split_poly_colors)):
    # Plot the polygon part
    x, y = split_polys.geoms[poly].exterior.xy
    plt.fill(x, y, color=split_poly_colors[poly], alpha=0.5)

#slice visPoly and plot
visPoly = shapeFiltering(visPoly,split_polys,split_poly_colors) #TODO: improve shapefiltering for intersection with visibility polygon2 based on raycasting
x, y = visPoly.exterior.xy
plt.plot(x, y, 'k-', label='Filtered Visibility Polygon')

#plot filtered free points
filteredFreePointCoords = pointFiltering(visPoly,start1,readings,angles,freePointIdx,occlusionVectorPoints)
first = True
for point in filteredFreePointCoords:
    if first:
        plt.scatter(point.x, point.y, color='green', marker='x', label='Filtered Points')
        first  = False
    else:
        plt.scatter(point.x, point.y, color='green', marker='x')

#find contrl law  components
freeArcsComponent,occlusionComponent = control_law2(readings,filteredFreePointCoords,occlusionVectorPoints)

# Normalize and plot the free arcs component
if np.linalg.norm(freeArcsComponent) != 0:
    normalized_free = freeArcsComponent / np.linalg.norm(freeArcsComponent)
    plt.arrow(start1[0], start1[1], normalized_free[0], normalized_free[1], 
              head_width=0.1, head_length=0.2, fc='blue', ec='blue', label='Free Arcs Component')
    
#TODO: Normalize and plot the occlusion arcs component

plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()

