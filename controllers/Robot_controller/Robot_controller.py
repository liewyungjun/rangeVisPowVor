from controller import Supervisor
import numpy as np

class RobotController:
    def __init__(self,x,y,size_x,size_y):
        self.supervisor = Supervisor()
        self.time_step = int(self.supervisor.getBasicTimeStep())
        self.robot_node = self.supervisor.getFromDef("ROBOT")
        self.x = x
        self.y = y
        self.size_x = size_x
        self.size_y = size_y
        #self.robot_node.getField('translation').setSFVec3f([self.x,self.y,2.0])
        self.wall_nodes = []  # Store wall nodes for updating
        self.gridIdx = 0
        def generate_grid(x_points, y_points, margin):
            grid = []
            for i in range(x_points):
                for j in range(y_points):
                    if i % 2 ==0:
                        grid.append([j * margin+margin/2, i * margin+margin/2])
                    else:
                        grid.append([(y_points-j-1) * margin+margin/2, i * margin+margin/2])
            return grid
        self.grid = generate_grid(5,5,3)


    def spawn_walls(self):
        wall_length=self.size_x
        wall_thickness=0.05
        center_z = 2.0
        wall_template = """
        DEF WALL%s Solid {
            translation %f %f %f
            children [
                Shape {
                    appearance Appearance {
                        material Material {
                            diffuseColor 0.8 0.8 0.8
                        }
                    }
                    geometry Box {
                        size %f %f %f
                    }
                }
            ]
        }
        """
        
        self.wall_nodes = []

        wall_positions = [
            (self.x - wall_length / 2, self.y, center_z,wall_thickness,wall_length),  # Left wall
            (self.x + wall_length / 2, self.y, center_z,wall_thickness,wall_length),  # Right wall
            (self.x, self.y  - wall_length / 2, center_z,wall_length,wall_thickness),  # Front wall
            (self.x, self.y  + wall_length / 2, center_z,wall_length,wall_thickness)   # Back wall
        ]
        for i, (x, y, z, a, b) in enumerate(wall_positions):
            wall_string = wall_template%(i,x,y,z,a,b,wall_thickness)
            self.supervisor.getRoot().getField("children").importMFNodeFromString(-1, wall_string)
            self.wall_nodes.append(self.supervisor.getFromDef(f"WALL{i}"))
   
    def update_walls(self):
        # Get current robot position
        current_pos = self.robot_node.getField('translation').getSFVec3f()
        if abs(current_pos[0]-self.x) > 0.01 or abs(current_pos[1] - self.y) > 0.01:
            self.x = current_pos[0]
            self.y = current_pos[1]
            # Remove existing walls
        #     wall_positions = [
        #     (self.x - self.size_x / 2, self.y, 2.0,0.05,self.size_x),  # Left wall
        #     (self.x + self.size_x / 2, self.y, 2.0,0.05,self.size_x),  # Right wall
        #     (self.x, self.y  - self.size_x / 2, 2.0,self.size_x,0.05),  # Front wall
        #     (self.x, self.y  + self.size_x / 2, 2.0,self.size_x,0.05)   # Back wall
        # ]
            for wall in range(len(self.wall_nodes)):
                # print(wall_positions[wall][:3])
                # self.wall_nodes[wall].getField('translation').setSFVec3f(np.array(wall_positions[wall][:3]))
                self.wall_nodes[wall].remove()
            # # Spawn new walls at updated position
            self.spawn_walls()

    def gridPattern(self):
        if self.gridIdx == len(self.grid)-1:
            return
        self.gridIdx +=1
        desiredPos = self.grid[self.gridIdx] + [2.0]
        self.robot_node.getField('translation').setSFVec3f(desiredPos)

    def run(self):
        count =0
        while self.supervisor.step(self.time_step) != -1:
            if count > 50:
                print("MOVING GRID")
                #self.gridPattern()
                count = 0
            self.update_walls()
            count +=1

if __name__ == "__main__":
    controller = RobotController(3.0, 3.0, 12.0,12.0)
    controller.spawn_walls()
    controller.run()



# def create_neighbor_marker(supervisor, position):
#     # Create sphere PROTO string
#     marker = supervisor.getRoot().getField('children')
#     marker.importMFNodeFromString(-1, '''
#     DEF NEIGHBOR_MARKER Solid {
#       translation %f %f %f
#       children [
#         Shape {
#           appearance PBRAppearance {
#             baseColor 1 1 0
#             emissiveColor 1 1 0
#           }
#           geometry Sphere {
#             radius 0.4
#           }
#         }
#       ]
#     }
#     ''' % (position[0], position[1], position[2]))

# Wall {
#   translation 0 -2.5 1.5
#   rotation 0 0 1 1.5708
#   name "wall(3)"
#   size 0.1 5 0.1
# }