import math
import matplotlib.pyplot as plt
import numpy as np
import os
this_dir, this_filename = os.path.split(__file__)
myfile = os.path.join(this_dir, 'lidar_reading.txt') 
max_range = 3.0  # Maximum sensor range to replace inf values
collision_readings_buffer = max_range*0.02 #2% for collision detection
occlusion_readings_buffer = max_range*0.05 #5% for occlusion detection
resolution = 540

def read_lidar_data():
    try:
        with open(myfile, 'r') as file:
            # Read lines and convert to float, replacing 'inf' with max_range
            readings = [0.0 for i in range(540)]
            for line in file:
                temp = line[1:-1].split(", ")
            for i in range(len(temp)):
                perm = 0.0
                if temp[i] == 'inf':
                    perm = max_range
                else:
                    perm = round(float(temp[i]), 3)
                readings[i]=perm
        return readings

    except FileNotFoundError:
        print("Error: lidar_reading.txt not found")
        return []

def control_law(readings,freePointIdx,occlusionVectorPoints):
    freeArcsComponent = [0.0,0.0]
    occlusionArcsComponent = [0.0,0.0]
    occlusionVectorPoints = occlusionVectorPoints[1:]

    # Create angles for each reading (assuming 360 degree scan)
    angles = np.linspace(0.5*np.pi, -1.5*np.pi, len(readings))

    # Free arcs component
    # freePointCoords = np.array([(readings[p] * np.cos(angles[p]),readings[p] * np.sin(angles[p]))\
    #                    for p in freePointIdx])
    freePointCoords = np.array([(np.cos(angles[p]),np.sin(angles[p]))\
                       for p in freePointIdx])
    freeArcsComponent = sum(freePointCoords)
    
    # Occlusion arcs component
    occlusionPointCoords = np.array([((readings[p[0]] * np.cos(angles[p[0]]),readings[p[0]] * np.sin(angles[p[0]])),\
                                      (readings[p[1]] * np.cos(angles[p[1]]),readings[p[1]] * np.sin(angles[p[1]])))\
                                        for p in occlusionVectorPoints])
    normal_vectors = np.array([(occlusionPointCoords[p][0][1] / readings[occlusionVectorPoints[p][0]],\
                       -occlusionPointCoords[p][0][0] / readings[occlusionVectorPoints[p][0]]) \
                        for p in range(len(occlusionPointCoords))])
    print("occlusionPointCoords")
    print(occlusionPointCoords)
    length_component = np.array([np.linalg.norm(p[1] - p[0]) for p in occlusionPointCoords])
    print("normals")
    print(normal_vectors)
    print("TIMES")
    print(length_component)
    print("EQUALS_")
    #occlusionArcsComponent = np.sum(normal_vectors * length_component[:, np.newaxis], axis=0)    
    occlusionArcsComponent = sum(np.array([[normal_vectors[p][0]*length_component[p],normal_vectors[p][1]*length_component[p]]for p in range(len(normal_vectors))]))
    print(occlusionArcsComponent)
    return freeArcsComponent,occlusionArcsComponent

def process_lidar_data(readings):
    freePointIdx = []
    occlusionVectorPoints = []
    for i in range(len(readings)):
        if readings[i] >= max_range - collision_readings_buffer: #no collision
            freePointIdx.append(i)
        if abs(readings[i] - readings[i-1]) > occlusion_readings_buffer:
            # print(f'adding point {i}')
            # print(f'because {readings[i]}-{readings[i-1]}')
            occlusionVectorPoints.append((i-1,i))
    return freePointIdx,occlusionVectorPoints


if __name__ == '__main__':
    # Read and plot the data
    readings = read_lidar_data()
    freePointIdx,occlusionVectorPoints = process_lidar_data(readings)
    freeArcsComponent,occlusionComponent = control_law(readings,freePointIdx,occlusionVectorPoints)
    print(f"Read in lidar data size {len(readings)}")
    #print(f'Free Arcs Indexes: {freePointIdx}')
    print(f'Occlusion Arcs Indexes: {occlusionVectorPoints}')
    print(f'Free Arcs Component: {freeArcsComponent}')
    print(f'Occlusion Arcs Component: {occlusionComponent}')
    #print(lidar_data)
