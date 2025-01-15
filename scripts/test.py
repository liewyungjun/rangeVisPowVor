
import numpy as np
import matplotlib.pyplot as plt
from lidar_processing import read_lidar_data
from shapely.geometry import Polygon,Point
# Read lidar readings from file
lidar_data = read_lidar_data()
# Convert polar coordinates (distance, angle) to cartesian (x,y)
angles = np.linspace(0.5*np.pi, -1.5*np.pi, len(lidar_data))
x = lidar_data * np.cos(angles)
y = lidar_data * np.sin(angles)
# Create points list and close the polygon by adding first point at end
points = list(zip(x, y))
points.append(points[0])  # Close the polygon
# Create Shapely polygon
polygon = Polygon(points)
x1, y1 = polygon.exterior.xy

polygon2 = Polygon([[1,-2.5],[1,-1],[3,-1],[3,-2.5]])
x2, y2 = polygon2.exterior.xy

result = polygon.difference(polygon2)
x3, y3 = result.exterior.xy

indexOfInterest = 20
print(points[indexOfInterest])
pointOfInterest = Point(points[indexOfInterest][0],points[indexOfInterest][1])

print(f'Point in visibility polygon: {polygon.contains(pointOfInterest)}')
print(f'Point in partitioned polygon: {result.contains(pointOfInterest)}')



plt.figure(figsize=(10, 10))
plt.plot(x1, y1, 'b-', label='Polygon 1')
plt.plot(x2, y2, 'b-', label='Polygon 2')
plt.plot(x3,y3,'r-',label='result')
#prin
plt.plot(pointOfInterest.x,pointOfInterest.y,'.',label='POI',markersize=20)# Create polygon by connecting points

#plt.plot(y,x, 'b-') #TODO: the angle progression is different
#plt.fill(x, y, alpha=0.3)
plt.grid(True)
plt.axis('equal')
plt.title('Lidar Polygon in Cartesian Coordinates')
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.legend()

plt.show()
