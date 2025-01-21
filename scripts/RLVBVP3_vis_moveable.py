
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from shapely import LineString
import RLVBVP3

class RLVBVP_Interactive_Vis:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.coords = [[2.2,0.0]]
        self.agent = RLVBVP3.RLVBVP3(pos=[0.0,0.0],comms_radius=6.0,
                      reading_radius=3.0,resolution=540,lidar_path='lidar_reading.txt')
        self.clickNeighbour = []
        # Connect mouse events
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.agentStep()
        self.plotAll()
        
    def on_click(self, event):
        if event.inaxes != self.ax or event.button !=3:
            return
        self.coords = [[event.xdata, event.ydata]]
        self.clickNeighbour = RLVBVP3.RLVBVP3(pos=[event.xdata, event.ydata],comms_radius=6.0,
                      reading_radius=3.0,resolution=540)
        self.agentStep()

        self.plotAll() # Clear and redraw
        
        plt.draw()

    def agentStep(self):
        self.agent.read_lidar_data() #read lidar data from file into self.readings
        self.agent.read_neighbors([self.clickNeighbour]) #update self.neighbour_coords
        self.agent.process_lidar_data() #find self.freePointIdx and self.occlusionArcs
        self.agent.visibilityPartitioning()
        self.agent.control_law() #find contrl law  components

    def plotAll(self):

        self.ax.clear()

        
                # Plot all reading coordinates
        # reading_x = [coord[0] for coord in self.agent.reading_coords]
        # reading_y = [coord[1] for coord in self.agent.reading_coords]
        # self.ax.scatter(reading_x, reading_y, color='green', marker='.', label='All Reading Points', s=10)


        self.ax.scatter(self.agent.pos[0], self.agent.pos[1], label='Drone 1')
        self.ax.scatter(self.agent.neighbour_coords[0][0], self.agent.neighbour_coords[0][1], label='Drone 2')
        
        sampledPoints = self.agent.freePointCoords[::3]
        first = True
        for point in sampledPoints:
            plt.scatter(point[0], point[1], color='green', marker='x', label='Reading Points' if first else "")
            first  = False
        
        occlusion_x = [[p.point1.x, p.point2.x] for p in self.agent.occlusionArcs]
        occlusion_y = [[p.point1.y, p.point2.y] for p in self.agent.occlusionArcs]
        # print(f"occlusions_x {occlusion_x}")
        # print(f"occlusions_y {occlusion_y}")
        plt.scatter(occlusion_x, occlusion_y, color='y',s=200,)
        for i in range(len(occlusion_x)):
            plt.plot([occlusion_x[i][0], occlusion_x[i][1]], [occlusion_y[i][0], occlusion_y[i][1]], 'y-', linewidth=2, label='Occlusion Line' if i == 0 else "")    

        for i in range(len(self.agent.obstacleLines)):
            x,y, = self.agent.obstacleLines[i].xy
            plt.plot(x,y,'m-', linewidth=2, label='obstacle Line' if i == 0 else "")

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

        filtered_free_x = [p.x for p in self.agent.filteredFreePointCoords]
        filtered_free_y = [p.y for p in self.agent.filteredFreePointCoords]
        plt.scatter(filtered_free_x, filtered_free_y, color='b',label='filtered free')

        # occ_filtered_free_x = [p.x for p in self.agent.filteredOcclusionCoords]
        # occ_filtered_free_y = [p.y for p in self.agent.filteredOcclusionCoords]
        # plt.scatter(occ_filtered_free_x, occ_filtered_free_y, color='m',label='filtered occ')

        for i in range(len(self.agent.occlusionArcs)):
            print(f'{i} arc: {len(self.agent.occlusionArcs[i].filtered_points)} filtered points')
            occ_x = [p.x for p in self.agent.occlusionArcs[i].filtered_points]
            occ_y = [p.y for p in self.agent.occlusionArcs[i].filtered_points]
            plt.scatter(occ_x, occ_y, color='r',label='filtered occ' if i ==0 else '')

        if np.linalg.norm(self.agent.freeArcsComponent) != 0:
            normalized_free = self.agent.freeArcsComponent / np.linalg.norm(self.agent.freeArcsComponent)
            plt.arrow(self.agent.pos[0], self.agent.pos[1], normalized_free[0], normalized_free[1], 
                    head_width=0.1, head_length=0.2, fc='blue', ec='blue', label='Free Arcs Component')
            
        if np.linalg.norm(self.agent.occlusionArcsComponent) != 0:
            normalized_free = self.agent.occlusionArcsComponent / np.linalg.norm(self.agent.occlusionArcsComponent)
            plt.arrow(self.agent.pos[0], self.agent.pos[1], normalized_free[0], normalized_free[1], 
                    head_width=0.1, head_length=0.2, fc='red', ec='red', label='Occ Arcs Component')

        totalComp = self.agent.freeArcsComponent + self.agent.occlusionArcsComponent  
        normalized_free = totalComp / np.linalg.norm(totalComp) / 2
        plt.arrow(self.agent.pos[0], self.agent.pos[1], normalized_free[0], normalized_free[1], 
                    head_width=0.1, head_length=0.2, fc='black', ec='black', label='Total Arcs Component')          
        
        print(f'Free arcs component: {self.agent.freeArcsComponent}')
        print(f'Occlusion arcs component: {self.agent.occlusionArcsComponent}')
        print(f'Total component: {totalComp}')
        print("---------------------------------")

        self.ax.legend(loc='lower right')
        self.ax.set_xlim(-7, 7)
        #self.ax.figure(figsize=(8,6))
        self.ax.set_ylim(-5,5)
        self.ax.set_aspect('equal')


if __name__ == "__main__":
    # Create and show the interactive plot
    vis = RLVBVP_Interactive_Vis()
    
    plt.show()
