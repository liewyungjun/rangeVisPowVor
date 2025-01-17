import numpy as np
import RLVBVP
import matplotlib.pyplot as plt

agent = RLVBVP.RLVBVP(pos=[0.0,0.0],comms_radius=6.0,
                      reading_radius=3.0,resolution=540)
neighbour_coords = [[2.2,0.0]]
agent.read_lidar_data()
agent.read_neighbor(neighbour_coords)
agent.process_lidar_data()
agent.generate_coordinatesWithPolygon()
agent.generate_neighbourCoordsWithPolygon()
agent.visibilityPartitioning()

#plot original visPoly and neighbour and intersection division
x, y = agent.visPoly.exterior.xy
plt.plot(x, y, 'b-', label='Visibility Polygon')
x, y = agent.neighbour_visPolys[0].exterior.xy
plt.plot(x, y, 'r-', label='Circle')
plt.scatter(agent.pos[0], agent.pos[1], label='Drone 1')
plt.scatter(agent.neighbour_coords[0][0], agent.neighbour_coords[0][1], label='Drone 2')
for poly in range(len(agent.intersectionPolysAllocation)):
    # Plot the polygon part
    x, y = agent.intersectionPolys.geoms[poly].exterior.xy
    plt.fill(x, y, color=agent.intersectionPolysAllocation[poly], alpha=0.5)

#slice visPoly and plot
agent.slice_visPoly() #TODO: improve shapefiltering for intersection with visibility polygon2 based on raycasting
x, y = agent.visPoly.exterior.xy
plt.plot(x, y, 'k-', label='Filtered Visibility Polygon')

#plot filtered free points
agent.filter_coord_points()
first = True
for point in agent.filteredFreePointCoords:
    if first:
        plt.scatter(point.x, point.y, color='green', marker='x', label='Filtered Points')
        first  = False
    else:
        plt.scatter(point.x, point.y, color='green', marker='x')

#find contrl law  components
agent.control_law()
# Normalize and plot the free arcs component
if np.linalg.norm(agent.freeArcsComponent) != 0:
    normalized_free = agent.freeArcsComponent / np.linalg.norm(agent.freeArcsComponent)
    plt.arrow(agent.pos[0], agent.pos[1], normalized_free[0], normalized_free[1], 
              head_width=0.1, head_length=0.2, fc='blue', ec='blue', label='Free Arcs Component')
    
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()
