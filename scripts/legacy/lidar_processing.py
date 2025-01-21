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

def control_law(readings,freeArcs,realOcclusionArcs):
    freeArcsComponent = [0.0,0.0]
    occlusionArcsComponent = [0.0,0.0]

    # Create angles for each reading (assuming 360 degree scan)
    angles = np.linspace(0, 2*np.pi, len(readings))

    # Free arcs component
    for arc in freeArcs:
        arcReadings = readings[arc[0]:arc[1]]
        start_angle = arc[0]/540 * 2*np.pi
        end_angle = arc[1]/540 * 2*np.pi
        obstacle_angles = np.linspace(start_angle,end_angle, len(arcReadings))
        # Calculate x and y components for each point in the arc
        x_components = np.cos(obstacle_angles)
        y_components = np.sin(obstacle_angles)

        for j in range(len(arcReadings)):
            freeArcsComponent[0] += x_components[j]
            freeArcsComponent[1] += y_components[j]
    
    # Occlusion arcs component
    for arc in realOcclusionArcs:
        arcReadings = readings[arc[0]:arc[1]+1]
        start_angle = arc[0]/540 * 2*np.pi
        end_angle = arc[1]/540 * 2*np.pi
        reflex_vector = (arcReadings[0] * np.cos(start_angle),arcReadings[0] * np.sin(start_angle))
        normal_vector = (reflex_vector[1]/arcReadings[0],-reflex_vector[0]/arcReadings[0])
        length_component = math.pow(arcReadings[1] - arcReadings[0],2)/(arcReadings[0])
        print("normal")
        print(normal_vector)
        print("length")
        print(length_component)
        #density_component = (1-)/2 #TODO: after visibility partitioning can check reflex vertex angle to find occlusion for other drone
        occlusionArcsComponent[0] += normal_vector[0] * length_component
        occlusionArcsComponent[1] += normal_vector[1] * length_component
    return freeArcsComponent,occlusionArcsComponent

def process_lidar_data(readings):
    freeArcs = []
    freeFlag = False
    for i in range(len(readings)):
        #There is a BUG here, it will not detect occlusion arcs or transitions
        #when the reading goes from far away obstacle to nearer obstacle
        #FIXED in the lidar_processing_points.py algo
        if readings[i] < max_range - collision_readings_buffer: #obstacle collision 
            if freeFlag: #transition from free to collision
                freeFlag = not freeFlag
                tempArc = (tempArc[0],i-1)
                freeArcs.append(tempArc)
        else: # no collision
            if not freeFlag:#transition from collision to free
                freeFlag = not freeFlag
                tempArc = (i,0)
    if freeFlag:
        tempArc = (tempArc[0],i)
        freeArcs.append(tempArc)
    realOcclusionArcs = []
    print(f"freeArcs: {freeArcs}")
    for j in freeArcs: 
        start = (j[0]-1,j[0])
        if j[1] == len(readings)-1:
            end = (j[1],0)
        else:
            end = (j[1],j[1]+1)
        print(f'looking for start and end occlusion to {j[0]} and {j[1]}')
        print(f'comparing {readings[j[0]]} and {readings[j[1]]}')
        if abs(readings[start[0]]-readings[start[1]])>occlusion_readings_buffer:
            realOcclusionArcs.append(start)
        if abs(readings[end[0]]-readings[end[1]])>occlusion_readings_buffer:
            print(f'comparing index {end[0]} and {end[1]} to {occlusion_readings_buffer}')
            realOcclusionArcs.append(end)
    print(f'real occlusion arcs: {realOcclusionArcs}')
    return freeArcs,realOcclusionArcs


if __name__ == '__main__':
    # Read and plot the data
    readings = read_lidar_data()
    freeArcs,realOcclusionArcs = process_lidar_data(readings)
    freeArcsComponent,occlusionComponent = control_law(readings,freeArcs,realOcclusionArcs)
    print(f"Read in lidar data size {len(readings)}")
    print(f'Free Arcs: {freeArcs}')
    print(f'Occlusion Arcs: {realOcclusionArcs}')
    print(f'Free Arcs Component: {freeArcsComponent}')
    print(f'Occlusion Arcs Component: {occlusionComponent}')
    #print(lidar_data)
