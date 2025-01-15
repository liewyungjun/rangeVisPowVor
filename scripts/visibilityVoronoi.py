import time
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
from scipy.spatial import Voronoi
# Starting positions
start_pos1 = Point(1, 0)
start_pos2 = Point(4.5, 0)
# Generate random points in polar coordinates within radius 2 for poly1
radius1 = 2
angles1 = np.sort(np.random.uniform(0, 2*np.pi, 5))  # Sorted for anticlockwise
# Convert to cartesian coordinates centered at start_pos1
poly1_coords = np.array([
    [start_pos1.x + radius1*np.cos(angles1[i]), start_pos1.y + radius1*np.sin(angles1[i])] 
    for i in range(5)
])
poly1_coords=np.array([[2.99646777, 0.11881259],
 [0.93761668, 1.99902684],
 [-0.87283525,  0.70177499],
 [0.91970083, -1.99838736],
 [1.01401592, -1.99995089]])

# Generate random points in polar coordinates within radius 2 for poly2
radius2 = 2
angles2 = np.sort(np.random.uniform(0, 2*np.pi, 5))  # Sorted for anticlockwise

# Convert to cartesian coordinates centered at start_pos2
poly2_coords = np.array([
    [start_pos2.x + radius2*np.cos(angles2[i]), start_pos2.y + radius2*np.sin(angles2[i])] 
    for i in range(5)
])
poly2_coords=([[ 5.04483673,  1.9243578 ],
 [ 3.06234852,  1.3903806 ],
 [ 2.92512092,  1.23278379],
 [ 2.51095012,  0.20899897],
 [ 5.55237156, -1.70073928]])
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
    minx, miny, maxx, maxy = intersection.bounds
    x = np.linspace(minx, maxx, 100)
    y = np.linspace(miny, maxy, 100)
    xx, yy = np.meshgrid(x, y)
    points = np.column_stack((xx.ravel(), yy.ravel()))
    
    # Keep only points within intersection
    mask = [Point(p).within(intersection) for p in points]
    points = points[mask]

    pointsColor = [''for p in points]
    for p in range(len(points)):
        dist1 = Point(points[p]).distance(start_pos1)
        dist2 = Point(points[p]).distance(start_pos2)
        pointsColor[p] = 'blue' if dist1 < dist2 else 'red'
    # Create Voronoi diagram for the two starting points
    #vor = Voronoi([start_pos1.coords[0], start_pos2.coords[0]])
    calculationtime = time.time()
    # Plot everything
    plt.figure(figsize=(10, 8))
    
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
    
    # Plot intersection points colored by nearest starting position   
    for p in range(len(pointsColor)):
        plt.plot(points[p][0], points[p][1], '.', color=pointsColor[p], markersize=1)

    plottingtime = time.time()
    
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()
    print(f'Intersection time: {intersectiontime-starttime}')
    print(f'Calculation time: {calculationtime-intersectiontime}')
    print(f'Plotting time: {plottingtime-calculationtime}')