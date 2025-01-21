
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from shapely import LineString
import RLVBVP2

class RLVBVP_Interactive_Vis:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.coords = [[2.2,0.0]]
        self.agent = RLVBVP2.RLVBVP2(pos=[0.0,0.0],comms_radius=6.0,
                      reading_radius=3.0,resolution=540,lidar_path='lidar_reading.txt')
        self.clickNeighbour = []
        # Connect mouse events
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.agentStep()
        self.plotAll()
        
    def on_click(self, event):
        if event.inaxes != self.ax:
            return
        self.coords = [[event.xdata, event.ydata]]
        self.clickNeighbour = RLVBVP2.RLVBVP2(pos=[event.xdata, event.ydata],comms_radius=6.0,
                      reading_radius=3.0,resolution=540)
        self.agentStep()

        self.plotAll() # Clear and redraw
        
        plt.draw()

    def agentStep(self):
        self.agent.read_lidar_data() #read lidar data from file into self.readings
        self.agent.read_neighbors([self.clickNeighbour]) #update self.neighbour_coords
        self.agent.process_lidar_data() #find self.freePointIdx and self.occlusionArcs
        self.agent.generate_coordinatesWithPolygon() #create self.reading_coords and self.visPoly with self.readings
        self.agent.generate_neighbourCoordsWithPolygon() #create self.neighbour_visPolys circle
        self.agent.visibilityPartitioning()
        self.agent.slice_visPoly() #TODO: work on assumption that neighbour vispoly is known
        self.agent.filter_coord_points()#TODO: add occlusion vector filtering
        # self.agent.control_law() #find contrl law  components

    def plotAll(self):

        self.ax.clear()

        
                # Plot all reading coordinates
        # reading_x = [coord[0] for coord in self.agent.reading_coords]
        # reading_y = [coord[1] for coord in self.agent.reading_coords]
        # self.ax.scatter(reading_x, reading_y, color='green', marker='.', label='All Reading Points', s=10)


        x, y = self.agent.visPoly.exterior.xy
        self.ax.plot(x, y, 'b-', label='Visibility Polygon')
        x, y = self.agent.neighbour_visPolys[0].exterior.xy
        self.ax.plot(x, y, 'r-', label='Circle')
        self.ax.scatter(self.agent.pos[0], self.agent.pos[1], label='Drone 1')
        self.ax.scatter(self.agent.neighbour_coords[0][0], self.agent.neighbour_coords[0][1], label='Drone 2')
        
        sampledPoints = self.agent.freePointIdx[::3]
        print(sampledPoints)
        first = True
        for point in sampledPoints:
            plt.scatter(self.agent.reading_coords[point][0], self.agent.reading_coords[point][1], color='green', marker='x', label='Reading Points' if first else "")
            first  = False
        
        occlusion_x = [[p.point1.x, p.point2.x] for p in self.agent.occlusionArcs]
        occlusion_y = [[p.point1.y, p.point2.y] for p in self.agent.occlusionArcs]
        # print(f"occlusions_x {occlusion_x}")
        # print(f"occlusions_y {occlusion_y}")
        plt.scatter(occlusion_x, occlusion_y, color='y',s=200,label='Occlusion Line')        
        for i in range(len(occlusion_x)):
            plt.plot([occlusion_x[i][0], occlusion_x[i][1]], [occlusion_y[i][0], occlusion_y[i][1]], 'y-', linewidth=2, label='Occlusion Line' if i == 0 else "")    

        for i in range(len(self.agent.obstacleLines)):
            x,y, = self.agent.obstacleLines[i].xy
            plt.plot(x,y,'m-', linewidth=2, label='obstacle Line' if i == 0 else "")

        neighbourX = [i.x for i in self.agent.neighbourPoints]
        neighbourY = [i.y for i in self.agent.neighbourPoints]
        plt.scatter(neighbourX,neighbourY)

        # Calculate midpoint
        midpoint_x = (self.agent.pos[0] + self.agent.neighbour_coords[0][0]) / 2
        midpoint_y = (self.agent.pos[1] + self.agent.neighbour_coords[0][1]) / 2

        # Calculate perpendicular slope (negative reciprocal)
        dx = self.agent.neighbour_coords[0][0] - self.agent.pos[0]
        dy = self.agent.neighbour_coords[0][1] - self.agent.pos[1]
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
        x,y = bisector_line.xy
        plt.plot(x,y, 'g--', label='Perpendicular Bisector')
        
        x,y = self.agent.neighbour_visPolys[0].exterior.xy
        plt.fill(x,y, 'g--', label='neighbour vispoly',color='lightcoral', alpha=0.5)

        #x, y = self.agent.intersectionPolys.exterior.xy
        #self.ax.plot(x, y, 'r-', label='Circle')
        #self.ax.fill(x, y, color='lightcoral', alpha=0.5)

        
        # #slice visPoly and plot
        if self.agent.slicedVisPoly.geom_type == 'Polygon':
            x, y = self.agent.slicedVisPoly.exterior.xy
            plt.plot(x, y, 'k-', label='Filtered Visibility Polygon')
        elif self.agent.slicedVisPoly.geom_type == 'MultiPolygon':
            print("VISPOLY IS MULTIPOLYGON")
            for j in range(len(self.agent.slicedVisPoly.geoms)):
                x, y = self.agent.slicedVisPoly.geoms[j].exterior.xy
                plt.plot(x, y, 'k-', label='Sliced Visibility Polygon' if j == 0 else "")

        # #plot filtered free points
        sampledPoints = self.agent.filteredFreePointCoords[::3]
        first = True
        for point in sampledPoints:
            plt.scatter(point.x, point.y, color='green', marker='x', label='Filtered Points' if first else "")
            first  = False

        # #plot filtered occlusion points
        occlusion_x = [[p.point1.x, p.point2.x] for p in self.agent.filteredOcclusionArcs]
        occlusion_y = [[p.point1.y, p.point2.y] for p in self.agent.filteredOcclusionArcs]
        print(f"occlusions_x {occlusion_x}")
        print(f"occlusions_y {occlusion_y}")
        plt.scatter(occlusion_x, occlusion_y, color='y',s=200,label='Occlusion Line')        
        for i in range(len(occlusion_x)):
            plt.plot([occlusion_x[i][0], occlusion_x[i][1]], [occlusion_y[i][0], occlusion_y[i][1]], 'y-', linewidth=2, label='Occlusion Line' if i == 0 else "")    
        
        # # Normalize and plot the free arcs component
        # if np.linalg.norm(self.agent.freeArcsComponent) != 0:
        #     normalized_free = self.agent.freeArcsComponent / np.linalg.norm(self.agent.freeArcsComponent)
        #     plt.arrow(self.agent.pos[0], self.agent.pos[1], normalized_free[0], normalized_free[1], 
        #             head_width=0.1, head_length=0.2, fc='blue', ec='blue', label='Free Arcs Component')
        self.ax.legend()

        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5,5)
        self.ax.set_aspect('equal')


if __name__ == "__main__":
    # Create and show the interactive plot
    vis = RLVBVP_Interactive_Vis()
    plt.show()
