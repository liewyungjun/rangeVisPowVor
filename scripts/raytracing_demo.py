from shapely import LineString
from shapely.geometry import Polygon, Point
from shapely.geometry import Point
import numpy as np
import matplotlib.pyplot as plt

# Create a polygon
with open('polygonPoints.txt', 'r') as file:
    polygon_points = [tuple(map(float, line.strip().split(','))) for line in file]
polygon = Polygon(polygon_points)

# Get the center of the polygon
center = Point([0.0,0.0])

# Create a buffer around the polygon
buffer = polygon.buffer(0.1)

# Plot the polygon
fig, ax = plt.subplots()
x, y = polygon.exterior.xy
polygon_line, = ax.plot(x, y, 'b')

# Plot the buffer
x, y = buffer.exterior.xy
ax.plot(x, y, 'g--')

# Plot the center of the polygon
ax.plot(center.x, center.y, 'ko')

def on_click(event):
    if event.button != 3 or event.inaxes != ax:
            return
    # Get the click location
    click_point = Point(event.xdata, event.ydata)
    
    # Check distances and change color if necessary
    new_colors = []
    ax.clear()
    count = 0
    intersect_count  = 0
    #print(polygon.exterior.coords.xy)
    for point in polygon.exterior.coords:
        #polygon_point = Point(point[0]*0.9,point[1]*0.9)
        polygon_point = Point(point[0],point[1])
        distance_to_center = polygon_point.distance(center)
        distance_to_click = polygon_point.distance(click_point)
        
        # if line.intersects(polygon):
        #     print("intersects")
        #     a,b = line.xy
        #     ax.plot(a,b)
        if distance_to_click < distance_to_center:
            new_colors.append('r')
            line = LineString([polygon_point,click_point])
            a,b = line.xy
            if line.intersects(buffer.boundary):
                intersectPoint = line.intersection(polygon)
                try:
                    print(intersectPoint.xy)
                    x,y, = intersectPoint.xy
                    ax.scatter(x,y,c='m',s=300)
                except NotImplementedError:
                    print(intersectPoint)
                    for i in intersectPoint.geoms:
                        x,y, = i.xy
                        ax.scatter(x,y,c='m',s=300)
                intersect_count +=1
                #print(f"{polygon_point.x},{polygon_point.y} intersects")
                ax.plot(a,b,c='y')
            else:
                ax.plot(a,b,c='g')
            count +=1
        else:
            new_colors.append('b')
    x, y = zip(*polygon.exterior.coords)
    ax.scatter(x, y, c=new_colors)
    x, y = polygon.exterior.xy
    polygon_line, = ax.plot(x, y, 'b')
    x, y = buffer.exterior.xy
    ax.plot(x, y, 'g--')
    ax.plot(center.x, center.y, 'ko')
    ax.plot(click_point.x, click_point.y, 'mo')
    print(f'Total nearer points: {count}')
    print(f'of which intersects: {intersect_count}')
    # Redraw the plot
    plt.draw()

# Connect the click event to the on_click function
fig.canvas.mpl_connect('button_press_event', on_click)

# Show the plot
plt.show()