import math
import matplotlib.pyplot as plt
import numpy as np
import os
from lidar_processing_points import * 
import logging
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union,split
from shapely.geometry import LineString

def visibilityPartitioning(visPoly,circle_poly,start1,start2):
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

