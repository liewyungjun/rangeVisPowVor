import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
from matplotlib.patches import Circle
from shapely import LineString
import RLVBVP_F2
#NOTE: that this tool only allows showing and interacting with 
#current sensing. This cannot show the current neighbours and partitioning in
#the simulation

target_id = 0
filepath=f'/home/dlserver/Documents/flush/controllers/fan_controller/webots{target_id}.txt'
#filepath = 'lidar_reading.txt'
class RLVBVP_Interactive_Vis:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.coords = [[1.1,0.0,0.0]]
        self.agent = RLVBVP_F2.RLVBVP_F2(pos=[0.0,0.0],comms_radius=6.0,
                      reading_radius=3.0,resolution=90,lidar_path=filepath,fov=60)
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
        self.agent.read_neighbors([[self.coords[0][0],self.coords[0][1],self.coords[0][2]-0.5*np.pi]]) #update self.neighbour_coords
        self.agent.process_lidar_data() #find self.freePointIdx and self.occlusionArcs
        self.agent.visibilityPartitioning()
        self.occ_length = self.agent.control_law() #find contrl law  components

    def plotAll(self):

        self.ax.clear()
        self.ax.scatter(self.agent.pos[0], self.agent.pos[1], label='Drone 1')

        sampledPoints = self.agent.reading_coords[::3]
        first = True
        for point in sampledPoints:
            plt.scatter(point[0], point[1], color='green', marker='x', label='Reading Points' if first else "")
            first  = False

        if self.agent.visPoly:
            x, y = self.agent.visPoly.exterior.xy
            points = np.column_stack((x, y))
            #self.ax.add_patch(plt.Polygon(points, alpha=0.7, color='blue', label='Visibility Polygon'))
        
        if self.agent.readingPoly:
            try:
                x, y = self.agent.readingPoly.exterior.xy
                points = np.column_stack((x, y))
                #print(points)
                self.ax.add_patch(plt.Polygon(points, alpha=0.7, color='red', label='reading Polygon'))        
            except AttributeError:
                for i in range(len(self.agent.readingPoly.geoms)):
                    if isinstance(self.agent.readingPoly.geoms[i], LineString):
                        continue                    
                    x, y = self.agent.readingPoly.geoms[i].exterior.xy
                    points = np.column_stack((x, y))
                    #points = np.vstack((points[:-1],[self.agent.pos]))
                    self.ax.add_patch(plt.Polygon(points, alpha=0.7, color='red', label='reading Polygon' if i == 0 else ' '))        

        if self.agent.avoidPolys:
            for i in range(len(self.agent.avoidPolys)):
                x, y = self.agent.avoidPolys[i].exterior.xy
                points = np.column_stack((x, y))
                self.ax.add_patch(plt.Polygon(points, alpha=0.7, color='green', label='avoid Polygon' if i ==0 else ' '))        

        if self.agent.neighbour_coords:
            self.ax.scatter(self.agent.neighbour_coords[0][0], self.agent.neighbour_coords[0][1], label='Drone 2')
            circle = plt.Circle((self.agent.neighbour_coords[0][0], self.agent.neighbour_coords[0][1]), 
                                            self.agent.reading_radius, fill=False, linestyle='--', color='gray',
                                            label='Sensing Radius')
            self.ax.add_patch(circle)

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

        self.ax.legend(loc='lower right')
        self.ax.set_xlim(-7, 7)
        #self.ax.figure(figsize=(8,6))
        self.ax.set_ylim(-5,5)
        self.ax.set_aspect('equal')

if __name__ == "__main__":
    # Create and show the interactive plot
    vis = RLVBVP_Interactive_Vis()
    
    plt.show()
