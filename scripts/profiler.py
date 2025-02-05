import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from shapely import LineString
import RLVBVP3
import cProfile
import pstats

target_id = 1
#filepath=f'/home/dlserver/Documents/flush/controllers/supervisor_controller/webots{target_id}.txt'
filepath = 'lidar_reading.txt'
class RLVBVP_PROFILE:
    def __init__(self):
        self.coords = [[np.random.uniform(-7, 7),np.random.uniform(-5, 5)]]
        self.agent = RLVBVP3.RLVBVP3(pos=[0.0,0.0],comms_radius=6.0,
                      reading_radius=3.0,resolution=540,lidar_path=filepath)
        #self.agentStep()

    def agentStep(self):
        start_time = time.time()
        
        if filepath[0] == '/':
            self.agent.read_lidar_data(path=filepath) #read lidar data from file into self.readings
        else:
            self.agent.read_lidar_data()
        self.coords = [[np.random.uniform(-7, 7),np.random.uniform(-5, 5)]]
        self.agent.read_neighbors(self.coords) #update self.neighbour_coords
        self.agent.process_lidar_data() #find self.freePointIdx and self.occlusionArcs
        self.agent.visibilityPartitioning()
        self.occ_length = self.agent.control_law() #find contrl law  components
def main():
    vis = RLVBVP_PROFILE()
    t1 = time.time()
    for i in range(100):
        if i % 10 ==0 :
            print(i)
        vis.agentStep()
    t2 = time.time()
    print(f'time taken for 100 steps: {t2-t1:.2f}')
    print(f'average step time: {(t2-t1)/100:.2f}')
    print(f'refresh rate = {100/(t2-t1):.2f} Hz')

if __name__ == "__main__":
    # profiler = cProfile.Profile()
    # profiler.enable()
    # main()
    # profiler.disable()
    # stats = pstats.Stats(profiler).sort_stats('cumulative')
    # stats.print_stats()
    main()