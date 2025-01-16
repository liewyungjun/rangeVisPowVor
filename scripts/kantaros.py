import math
import matplotlib.pyplot as plt
import numpy as np
import os
from lidar_processing_points import * 
import logging
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union,split
from shapely.geometry import LineString

start1= [0.0,0.0]
start2 = [2.2,0]
radius1 = 3.0
radius2 = 3.0

readings = read_lidar_data()
freePointIdx,occlusionVectorPoints = process_lidar_data(readings)
angles = np.linspace(0.5*np.pi, -1.5*np.pi, len(readings))
poly_coords = np.array([[readings[i]*np.cos(angles[i])+start1[0],start1[1]+readings[i]*np.sin(angles[i])] for i in range(len(readings))])
visPoly = Polygon(poly_coords)

circle_coords = np.array([[np.cos(theta)*radius2+start2[0], np.sin(theta)*radius2+start2[1]] for theta in np.linspace(0, 2*np.pi, 100)])
circle_poly = Polygon(circle_coords)

def visibilityPartitioning(visPoly,circle_poly):
    #find intersection polygon and midpoint
    intersection = visPoly.intersection(circle_poly)
    midpoint_x = (start1[0] + start2[0]) / 2
    midpoint_y = (start1[1] + start2[1]) / 2

    # Calculate perpendicular vector
    # Original vector is (start2.x - start1.x, start2.y - start1.y)
    # Perpendicular vector is (-dy, dx)
    dx = start2[0] - start1[0]
    dy = start2[1] - start1[1]
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
        dist_to_start1 = Point(start1[0], start1[1]).distance(centroid)
        dist_to_start2 = Point(start2[0], start2[1]).distance(centroid)
        
        # Color based on closest starting point
        split_poly_colors[poly] = 'lightblue' if dist_to_start1 < dist_to_start2 else 'lightcoral'
    return split_polys,split_poly_colors

def shapeFiltering(visPoly,split_polys,split_poly_colors):
    for i in range(len(split_poly_colors)):
        if split_poly_colors[i] == 'lightcoral':
            visPoly = visPoly.difference(split_polys.geoms[i])
            #print("filtering shape")
    return visPoly

def pointFiltering(visPoly,startingPoint,readings,angles,freePointIdx,occlusionVectorPoints):
    filteredFreePointCoords = []
    for i in freePointIdx:
        pointLocation = Point(0.9999*readings[i]*np.cos(angles[i])+startingPoint[0],0.9999*readings[i]*np.sin(angles[i])+startingPoint[1])        
        if visPoly.contains(pointLocation):
            filteredFreePointCoords.append(pointLocation)
    #TODO: how to filter out occlusion points
    return filteredFreePointCoords




split_polys,split_poly_colors = visibilityPartitioning(visPoly,circle_poly)

plt.figure()
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


visPoly = shapeFiltering(visPoly,split_polys,split_poly_colors) #TODO: improve shapefiltering for intersection with visibility polygon2
x, y = visPoly.exterior.xy
plt.plot(x, y, 'k-', label='Filtered Visibility Polygon')

filteredFreePointCoords = pointFiltering(visPoly,start1,readings,angles,freePointIdx,occlusionVectorPoints)
first = True
for point in filteredFreePointCoords:
    if first:
        plt.scatter(point.x, point.y, color='green', marker='x', label='Filtered Points')
        first  = False
    else:
        plt.scatter(point.x, point.y, color='green', marker='x')

freeArcsComponent,occlusionComponent = control_law2(readings,filteredFreePointCoords,occlusionVectorPoints)

# Normalize and plot the free arcs component
if np.linalg.norm(freeArcsComponent) != 0:
    normalized_free = freeArcsComponent / np.linalg.norm(freeArcsComponent)
    plt.arrow(start1[0], start1[1], normalized_free[0], normalized_free[1], 
              head_width=0.1, head_length=0.2, fc='blue', ec='blue', label='Free Arcs Component')
    
#TODO: Normalize and plot the free arcs component

plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()

