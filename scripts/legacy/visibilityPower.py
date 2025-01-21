import time
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union,split
from shapely.geometry import LineString

from scipy.spatial import Voronoi
# Starting positions
start_pos1 = Point(1, 0)
start_pos2 = Point(4.5, 0)
# Generate random points in polar coordinates within radius 2 for poly1
radius1 = 4
angles1 = np.sort(np.random.uniform(0, 2*np.pi, 5))  # Sorted for anticlockwise
# Convert to cartesian coordinates centered at start_pos1
poly1_coords = np.array([
    [start_pos1.x + radius1*np.cos(angles1[i]), start_pos1.y + radius1*np.sin(angles1[i])] 
    for i in range(5)
])
poly1_coords=[[ 4.97571371,  0.4401142 ],
 [0,3.9],
 [-2.61926196, -1.70321546],
 [-0.694636,   -3.62328702],
 [ 2.96875915, -3.48195167]]

# Generate random points in polar coordinates within radius 2 for poly2
radius2 = 4
angles2 = np.sort(np.random.uniform(0, 2*np.pi, 5))  # Sorted for anticlockwise

# Convert to cartesian coordinates centered at start_pos2
poly2_coords = np.array([
    [start_pos2.x + radius2*np.cos(angles2[i]), start_pos2.y + radius2*np.sin(angles2[i])] 
    for i in range(5)
])

poly2_coords=[[ 0.5,0],
 [5,4],
 [8,-1.8],
 [6.694636,   -3.62328702],
 [ 2, -3.48195167]]
# Create Shapely polygons
polygon1 = Polygon(poly1_coords)
polygon2 = Polygon(poly2_coords)



starttime = time.time()
# Find intersection
intersection = polygon1.intersection(polygon2)
intersectiontime = time.time()

# Create a fine grid of points in the intersection area
if not intersection.is_empty:
    print("intersection is not empty")
    print(f'Poly1 points: {poly1_coords}')
    print(f'Poly2 points: {poly2_coords}')    
    # Plot everything
    # Find midpoint between start positions
    midpoint_x = (start_pos1.x + start_pos2.x) / 2
    midpoint_y = (start_pos1.y + start_pos2.y) / 2
    
    # Calculate perpendicular vector
    # Original vector is (start_pos2.x - start_pos1.x, start_pos2.y - start_pos1.y)
    # Perpendicular vector is (-dy, dx)
    dx = start_pos2.x - start_pos1.x
    dy = start_pos2.y - start_pos1.y
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
        dist_to_start1 = Point(start_pos1.x, start_pos1.y).distance(centroid)
        dist_to_start2 = Point(start_pos2.x, start_pos2.y).distance(centroid)
        
        # Color based on closest starting point
        split_poly_colors[poly] = 'lightblue' if dist_to_start1 < dist_to_start2 else 'lightcoral'

    calculationtime = time.time()


    plt.figure(figsize=(10, 8))

    # Plot the line between start points
    plt.plot([start_pos1.x, start_pos2.x], [start_pos1.y, start_pos2.y], 'g--', label='Connection')
        
    # Plot midpoint
    plt.plot(midpoint_x, midpoint_y, 'go', markersize=8, label='Midpoint')
    
    # Plot perpendicular vector
    plt.plot([midpoint_x - perp_dx, midpoint_x + perp_dx], 
                [midpoint_y - perp_dy, midpoint_y + perp_dy], 
                'g-', label='Perpendicular')
    
    
        
    for poly in range(len(split_poly_colors)):
        # Plot the polygon part
        x, y = split_polys.geoms[poly].exterior.xy
        plt.fill(x, y, color=split_poly_colors[poly], alpha=0.5)

    
    # Plot polygons
    x1, y1 = polygon1.exterior.xy
    x2, y2 = polygon2.exterior.xy
    plt.plot(x1, y1, 'b-', label='Polygon 1')
    plt.plot(x2, y2, 'r-', label='Polygon 2')
    # Plot circles
    circle1 = plt.Circle((start_pos1.x, start_pos1.y), radius1, color='b', fill=False, linestyle='--')
    circle2 = plt.Circle((start_pos2.x, start_pos2.y), radius2, color='r', fill=False, linestyle='--')
    plt.gca().add_patch(circle1)
    plt.gca().add_patch(circle2)
    
    # Plot starting positions
    plt.plot(start_pos1.x, start_pos1.y, 'bo', markersize=10, label='Start 1')
    plt.plot(start_pos2.x, start_pos2.y, 'ro', markersize=10, label='Start 2')
    
    plottingtime = time.time()

    
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()
    print(f'Intersection time: {intersectiontime-starttime}')
    print(f'Calculation time: {calculationtime-intersectiontime}')
    print(f'Plotting time: {plottingtime-calculationtime}')
