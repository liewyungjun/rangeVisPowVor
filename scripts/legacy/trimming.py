import numpy as np
from shapely.geometry import Polygon
import matplotlib.pyplot as plt

# Create a large 50-sided regular polygon
theta = np.linspace(0, 2*np.pi, 50, endpoint=False)
large_radius = 10
large_x = large_radius * np.cos(theta)
large_y = large_radius * np.sin(theta)
large_polygon = Polygon(zip(large_x, large_y))

# Create a smaller 10-sided regular polygon, slightly offset
small_radius = 4
theta_small = np.linspace(0, 2*np.pi, 10, endpoint=False)
small_x = small_radius * np.cos(theta_small) + 5  # offset by 5 units
small_y = small_radius * np.sin(theta_small) + 5  # offset by 5 units
small_polygon = Polygon(zip(small_x, small_y))
small_polygon = Polygon([[2.5,-10],[2.5,10],[5,10],[5,-10],[2.5,-10]])
# Get the difference between the polygons
result = large_polygon.difference(small_polygon)
#print(large_polygon)
print(result)

# Plot the polygons
plt.figure(figsize=(10, 10))
x, y = large_polygon.exterior.xy
#plt.plot(x, y, 'b-', label='Large Polygon')
x, y = small_polygon.exterior.xy
#plt.plot(x, y, 'r-', label='Small Polygon')
x, y = result.geoms[0].exterior.xy
plt.plot(x, y, 'g--', label='Result')
x, y = result.geoms[1].exterior.xy
plt.plot(x, y, 'g--', label='Result')
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.show()
