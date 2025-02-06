import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
from matplotlib.patches import Circle
from shapely import LineString
import RLVBVP_F
#NOTE: that this tool only allows showing and interacting with 
#current sensing. This cannot show the current neighbours and partitioning in
#the simulation

target_id = 0
filepath=f'/home/dlserver/Documents/flush/controllers/fan_controller/webots{target_id}.txt'
#filepath = 'lidar_reading.txt'
class RLVBVP_Interactive_Vis:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.coords = [[0.0,0.0,0.0]]
        self.agent = RLVBVP_F.RLVBVP_F(pos=[0.0,0.0],comms_radius=6.0,
                      reading_radius=3.0,resolution=90,lidar_path=filepath,fov=60)
        #self.clickNeighbour = []
        # Connect mouse events
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.agentStep()
        self.plotAll()
        self.occ_length = 1
        
    def on_click(self, event):
        if event.inaxes != self.ax or event.button !=3:
            if event.button ==1:
                angle = np.arctan2(event.ydata - self.coords[0][1], event.xdata - self.coords[0][0])
                self.coords = [[self.coords[0][0], self.coords[0][1], angle]] if self.coords else [[event.xdata, event.ydata, angle]]
                print(f'Left clicked: now neighbour is {self.coords}')
                self.agentStep()
                self.plotAll() # Clear and redraw
                plt.draw()
            return
        self.coords = [[event.xdata, event.ydata,0.0]]
        self.agentStep()
        self.plotAll() # Clear and redraw
        plt.draw()

    def agentStep(self):
        if filepath[0] == '/':
            self.agent.read_lidar_data(path=filepath) #read lidar data from file into self.readings
        else:
            self.agent.read_lidar_data()
        
        #print(f'read in: {self.agent.readings}')
        #self.agent.get_lidar_data([3.0,3.0,3.0,3.0])
        
        self.agent.read_neighbors([[self.coords[0][0],self.coords[0][1],self.coords[0][2]-0.5*np.pi]]) #update self.neighbour_coords
        self.agent.process_lidar_data() #find self.freePointIdx and self.occlusionArcs
        self.agent.visibilityPartitioning()
        self.occ_length = self.agent.control_law() #find contrl law  components

    def plotAll(self):

        self.ax.clear()
        self.ax.scatter(self.agent.pos[0], self.agent.pos[1], label='Drone 1')

        # sampledPoints = np.concatenate((self.agent.leftCone[::3], self.agent.rightCone[::3]))
        # first = True
        # for point in sampledPoints:
        #     plt.scatter(point[0], point[1], color='green', marker='x', label='Cone Points' if first else "")
        #     first  = False
        
        sampledPoints = self.agent.freePointCoords[::3]
        first = True
        for point in sampledPoints:
            plt.scatter(point[0], point[1], color='green', marker='x', label='Reading Points' if first else "")
            first  = False
        
        occlusion_x = [[p.point1.x, p.point2.x] for p in self.agent.occlusionArcs]
        occlusion_y = [[p.point1.y, p.point2.y] for p in self.agent.occlusionArcs]
        plt.scatter(occlusion_x, occlusion_y, color='y',s=200,)
        for i in range(len(occlusion_x)):
            plt.plot([occlusion_x[i][0], occlusion_x[i][1]], [occlusion_y[i][0], occlusion_y[i][1]],
                      'y-', linewidth=2, label='Occlusion Line' if i == 0 else "")   

        cone_x = [[p.point1.x, p.point2.x] for p in self.agent.coneCoords]
        cone_y = [[p.point1.y, p.point2.y] for p in self.agent.coneCoords]
        plt.scatter(cone_x, cone_y, color='y',s=200,)
        for i in range(len(cone_x)):
            plt.plot([cone_x[i][0], cone_x[i][1]], [cone_y[i][0], cone_y[i][1]],
                      'y-', linewidth=2, label='Cone Line' if i == 0 else "")    

        for i in range(len(self.agent.obstacleLines)):
            x,y, = self.agent.obstacleLines[i].xy
            plt.plot(x,y,'m-', linewidth=2, label='obstacle Line' if i == 0 else "")

        if self.agent.neighbour_coords:
            self.ax.scatter(self.agent.neighbour_coords[0][0], self.agent.neighbour_coords[0][1], label='Drone 2')
            circle = plt.Circle((self.agent.neighbour_coords[0][0], self.agent.neighbour_coords[0][1]), 
                                            self.agent.reading_radius, fill=False, linestyle='--', color='gray',
                                            label='Sensing Radius')
            self.ax.add_patch(circle)
            neighbour_angle = self.coords[0][2]/(2 * np.pi) * 360
            wedge = Wedge((self.agent.neighbour_coords[0][0], self.agent.neighbour_coords[0][1]), 
                          self.agent.reading_radius, neighbour_angle-self.agent.fov/2, 
                          neighbour_angle + self.agent.fov/2, facecolor='orange', alpha=0.5)
            self.ax.add_patch(wedge)

            # Plot unit vector showing neighbor's heading
            angle_rad = self.agent.neighbour_coords[0][2]# Convert to radians and adjust for plotting
            dx = np.cos(angle_rad)
            dy = np.sin(angle_rad)
            plt.arrow(self.agent.neighbour_coords[0][0], self.agent.neighbour_coords[0][1], 
                        dx, dy, head_width=0.1, head_length=0.2, fc='red', ec='red', 
                        label='Neighbor Heading')
            

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

        for i in range(len(self.agent.occlusionArcs)):
            print(f'{i} arc: {len(self.agent.occlusionArcs[i].filtered_points)} filtered points')
            occ_x = [p.x for p in self.agent.occlusionArcs[i].filtered_points]
            occ_y = [p.y for p in self.agent.occlusionArcs[i].filtered_points]
            plt.scatter(occ_x, occ_y, color='r',label='filtered occ' if i ==0 else '')

        for i in range(len(self.agent.coneCoords)):
            print(f'{i} arc: {len(self.agent.coneCoords[i].filtered_points)} filtered cone points')
            occ_x = [p.x for p in self.agent.coneCoords[i].filtered_points]
            occ_y = [p.y for p in self.agent.coneCoords[i].filtered_points]
            plt.scatter(occ_x, occ_y, color='c',label='filtered cone' if i ==0 else '')

        if np.linalg.norm(self.agent.freeArcsComponent) != 0:
            normalized_free = self.agent.freeArcsComponent / np.linalg.norm(self.agent.freeArcsComponent)
            plt.arrow(self.agent.pos[0], self.agent.pos[1], normalized_free[0], normalized_free[1], 
                    head_width=0.1, head_length=0.2, fc='blue', ec='blue', label='Free Arcs Component')
            
        if np.linalg.norm(self.agent.occlusionArcsComponent) != 0:
            normalized_free = self.agent.occlusionArcsComponent / np.linalg.norm(self.agent.occlusionArcsComponent)
            plt.arrow(self.agent.pos[0], self.agent.pos[1], normalized_free[0], normalized_free[1], 
                    head_width=0.1, head_length=0.2, fc='red', ec='red', label='Occ Arcs Component')

        if np.linalg.norm(self.agent.coneComponent) != 0:
            normalized_free = self.agent.coneComponent / np.linalg.norm(self.agent.coneComponent)
            plt.arrow(self.agent.pos[0], self.agent.pos[1], normalized_free[0], normalized_free[1], 
                    head_width=0.1, head_length=0.2, fc='green', ec='green', label='Cone Arcs Component')

        totalComp = self.agent.freeArcsComponent + self.agent.occlusionArcsComponent + self.agent.coneComponent
        normalized_free = totalComp / np.linalg.norm(totalComp) / 2
        plt.arrow(self.agent.pos[0], self.agent.pos[1], normalized_free[0], normalized_free[1], 
                    head_width=0.1, head_length=0.2, fc='black', ec='black', label='Total Arcs Component')          
        
        # print(f'Free arcs component: {self.agent.freeArcsComponent}')
        # print(f'Occlusion arcs component: {self.agent.occlusionArcsComponent}')
        # print(f'cone arcs component: {self.agent.coneComponent}')
        # print(f'Total component: {totalComp}')
        # print(f'Free arcs magnitude: {np.linalg.norm(self.agent.freeArcsComponent)}')
        # print(f'Occlusion arcs magnitude: {np.linalg.norm(self.agent.occlusionArcsComponent)}')
        # print(f'Free arcs magnitude to length ratio: {np.linalg.norm(self.agent.freeArcsComponent) / len(self.agent.filteredFreePointCoords)}')
        # print(f'Occlusion arcs magnitude to length ratio: {np.linalg.norm(self.agent.occlusionArcsComponent) / self.occ_length}')
        # print("---------------------------------")

        self.ax.legend(loc='lower right')
        self.ax.set_xlim(-7, 7)
        #self.ax.figure(figsize=(8,6))
        self.ax.set_ylim(-5,5)
        self.ax.set_aspect('equal')

if __name__ == "__main__":
    # Create and show the interactive plot
    vis = RLVBVP_Interactive_Vis()
    
    plt.show()
