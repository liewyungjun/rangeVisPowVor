import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

class LinePlotter:
    def __init__(self, resolution=101):
        self.fig, self.ax = plt.subplots()
        self.resolution = resolution
        self.x = np.linspace(0, 10, resolution)
        #print(len(self.x))
        #print(self.x[-1])
        #print(self.x[100])
        self.y = self.x  # Simple linear relationship
        
        # Initialize points of interest
        self.poi_indices = [10, resolution//6, resolution-1]  # Track indices in self.x and self.y
        self.poi_x = [self.x[i] for i in self.poi_indices]
        self.poi_y = [self.y[i] for i in self.poi_indices]
        
        
        # Plot the line and points
        self.line, = self.ax.plot(self.x, self.y, 'b-')
        self.points, = self.ax.plot(self.poi_x, self.poi_y, 'ro', picker=5)
        
        # Add labels for points
        labels = ['pr', 'pa', 'pb']
        for i, (x, y, label) in enumerate(zip(self.poi_x, self.poi_y, labels)):
            self.ax.annotate(label, (x, y), xytext=(5,0), textcoords='offset points')
        
        # Connect event handlers
        # self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        # self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)

        a_idxArr = []
        freeArr = []
        occArr = []
        a_idx = self.poi_indices[0]
        print(self.poi_x)
        print(self.poi_y)
        for i in range(90):
            temp1,temp2 = self.control_law(printt = False)
            
            self.poi_x[1] = self.x[a_idx]
            self.poi_y[1] = self.y[a_idx]
            self.poi_indices[1] = a_idx
            freeArr.append(temp1)
            occArr.append(temp2)
            a_idxArr.append(a_idx)
            a_idx +=1
        #Create a new figure for the magnitude plot
        fig2, ax2 = plt.subplots()
        ax2.plot([self.x[i] for i in a_idxArr], [np.linalg.norm(i) for i in freeArr], 'g-', label='Free Arc Comp')
        ax2.plot([self.x[i] for i in a_idxArr], [np.linalg.norm(i) for i in occArr], 'r-', label='Occ Arc Comp')
        ax2.set_xlabel('X_loc')
        ax2.set_ylabel('Magnitude')
        ax2.set_title('Arc Comps against Pa position')
        ax2.axvline(x=self.poi_x[0], color='red', label='Pr', linestyle='--')
        ax2.axvline(x=self.poi_x[2], color='green', label='Pb', linestyle='--')        
        ax2.legend()
        plt.show()

        
        
        self.selected_point = None
        self.ax.set_xlim(-1, 11)
        self.ax.set_ylim(-1, 11)
        
    def find_closest_point(self, x, y):
        distances = np.sqrt((np.array(self.poi_x) - x)**2 + 
                          (np.array(self.poi_y) - y)**2)
        return np.argmin(distances)
    
    def find_nearest_line_point(self, x, y):
        # Find the nearest point on the line to the given coordinates
        distances = np.sqrt((self.x - x)**2 + (self.y - y)**2)
        nearest_idx = np.argmin(distances)
        return self.x[nearest_idx], self.y[nearest_idx], nearest_idx

    def on_click(self, event):
        if event.inaxes != self.ax:
            return
        if event.button == 1:  # Left click
            if self.selected_point is None:
                # First click - select nearest point
                self.selected_point = self.find_closest_point(event.xdata, event.ydata)
            else:
                # Second click - snap to nearest point on line
                nearest_x, nearest_y, nearest_idx = self.find_nearest_line_point(event.xdata, event.ydata)
                self.poi_x[self.selected_point] = nearest_x
                self.poi_y[self.selected_point] = nearest_y
                self.poi_indices[self.selected_point] = nearest_idx
                self.points.set_data(self.poi_x, self.poi_y)
                # Clear existing texts
                for text in self.ax.texts:
                    text.remove()
                # Replot labels
                labels = ['pr', 'pa', 'pb']
                for i, (x, y, label) in enumerate(zip(self.poi_x, self.poi_y, labels)):
                    self.ax.annotate(label, (x, y), xytext=(5,0), textcoords='offset points')
                self.fig.canvas.draw_idle()
                self.selected_point = None    
                print(f'Point_x: {self.poi_x}')
                print(f'Point_y: {self.poi_y}')
                print(f'Point_indices: {self.poi_indices}')
                self.control_law()
                print("-----------------------------------------------")
            self.ax.scatter(0.0, 0.0, color='green', marker='.', label='Drone Pos', s=10)
    def on_motion(self, event):
        if event.inaxes != self.ax or self.selected_point is None:
            return
        
        # Update point position to follow mouse
        self.poi_x[self.selected_point] = event.xdata
        self.poi_y[self.selected_point] = event.ydata
            
        # Update the plot and label position
        self.points.set_data(self.poi_x, self.poi_y)
        #labels = ['pr', 'pa', 'pb']
        #self.ax.texts[self.selected_point].set_position((event.xdata, event.ydata))
        self.fig.canvas.draw_idle()    

    def control_law(self,printt = True):

        freeArcsComponentArr = np.zeros(2)
        for i in range(self.poi_indices[1], self.poi_indices[2] + 1):
            # Calculate direction vector from robot to point
            point = np.array([self.x[i],self.y[i]])
            # Normalize to unit vector
            norm = np.linalg.norm(point)
            if norm > 0:
                unit_vector = point / norm
                freeArcsComponentArr += unit_vector
        

        readingPerUnitLength = 10
        pr = np.array([self.poi_x[0],self.poi_y[0]])
        pa = np.array([self.poi_x[1],self.poi_y[1]])
        pb = np.array([self.poi_x[2],self.poi_y[2]])
        densityComponent = (1-math.pow(np.linalg.norm(pa-pr)/np.linalg.norm(pb-pr),2))/2
        normal = np.array([-1.0,1.0])

        #occlusionArcsComponentArr = normal * math.pow(np.linalg.norm(pb-pr),2) / np.linalg.norm(pr) * densityComponent * readingPerUnitLength
        occlusionArcsComponentArr = normal * math.pow(np.linalg.norm(pb-pr),2) / np.linalg.norm(pr-np.zeros(2)) * densityComponent * readingPerUnitLength

        if printt:
            print(f'free num points: {self.poi_indices[2] - self.poi_indices[1] + 1}')
            print(f'free arcs comp : {freeArcsComponentArr}')
            print(f'free arcs magnitude: {np.linalg.norm(freeArcsComponentArr)}')
            print(f'free arcs ratio: {np.linalg.norm(freeArcsComponentArr) / (self.poi_indices[2] - self.poi_indices[1] + 1)}')
            print(f'Density component:"{densityComponent}')
            print(f'occ num points: {self.poi_indices[2] - self.poi_indices[1] + 1}')
            print(f'occ arcs comp : {occlusionArcsComponentArr}')
            print(f'occ arcs magnitude: {np.linalg.norm(occlusionArcsComponentArr)}')
            print(f'occ arcs ratio: {np.linalg.norm(occlusionArcsComponentArr) / (self.poi_indices[2] - self.poi_indices[1] + 1)}')


        # for i in self.occlusionArcs:
        #     occlusionLine = i.filteredLineString
        #     if len(i.filtered_points) > 2:
        #         # Get coordinates of all points on the line
        #         coords = np.array(occlusionLine.coords)
        #         occ_length += len(coords)
        #         # Find nearest and furthest points
        #         distances = [Point(self.pos[0], self.pos[1]).distance(Point(x, y)) for x, y in coords]
        #         nearest_idx = np.argmin(distances)
        #         furthest_idx = np.argmax(distances)
        #         #print(f'nearest: {nearest_idx}, furthest: {furthest_idx}')
                
        #         pa = coords[nearest_idx]
        #         pb = coords[furthest_idx]
        #         pr = np.array([i.reflex.x,i.reflex.y])
                
        #         # Calculate direction vector of the line
        #         line_vector = coords[-1] - coords[0]
                
        #         # Calculate normal vector (rotate 90 degrees counterclockwise)
        #         normal = np.array([-line_vector[1], line_vector[0]])
                
        #         # Normalize the normal vector
        #         normal = normal / np.linalg.norm(normal)
                
        #         # Ensure normal points inward by checking if it points towards self.pos
        #         center_to_line = np.array([self.pos[0], self.pos[1]]) - coords[0]
        #         if np.dot(normal, center_to_line) < 0:
        #             normal = -normal
                
        #         densityComponent = (1-math.pow(np.linalg.norm((pa-pr)/(pb-pr)),2))/2
        #         #print(f'density component: {densityComponent}')
        #         tempComponent = normal * math.pow(np.linalg.norm(pb-pa),2) / (np.linalg.norm(pr-self.pos) * densityComponent)
        #         #print(f'temp component: {normal} * {math.pow(np.linalg.norm(pb-pa),2)} / {(np.linalg.norm(pr-self.pos) * densityComponent)}')
        #         #print(f'adding temp occ component: {tempComponent}')
        #         occlusionArcsComponentArr += tempComponent * readingPerUnitLength
        # print(f'occ length: {occ_length}')
        # self.occlusionArcsComponent = occlusionArcsComponentArr
        return freeArcsComponentArr,occlusionArcsComponentArr

    def show(self):
        plt.show()

# Create and show the plot
plotter = LinePlotter()
plotter.show()
