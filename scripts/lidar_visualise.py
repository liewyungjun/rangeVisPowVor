import math
import matplotlib.pyplot as plt
import numpy as np
import os
from lidar_processing import *

def plot_lidar_data(readings,freeArcs,realOcclusionArcs,freeArcsComponent,occlusionComponent):    
    # Create angles for each reading (assuming 360 degree scan)
    angles = np.linspace(0, 2*np.pi, len(readings))
    
    # Convert to polar coordinates
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='polar')
    
    # Plot the original data
    ax.plot(angles, readings, 'r-', label='Raw Readings (Collisions)')
    
    # Plot obstacle arcs in red
    for arc in freeArcs:
        arcReadings = readings[arc[0]:arc[1]]
        start_angle = arc[0]/540 * 2*np.pi
        end_angle = arc[1]/540 * 2*np.pi
        obstacle_angles = np.linspace(start_angle,end_angle, len(arcReadings))
        if arc == freeArcs[0]:
            ax.plot(obstacle_angles, arcReadings, 'b-', linewidth=2, label='Free Arcs')
        else:
            ax.plot(obstacle_angles, arcReadings, 'b-', linewidth=2)
    
    # Plot occlusion arcs in green
    for arc in realOcclusionArcs:
        arcReadings = readings[arc[0]:arc[1]+1]
        print(f"occlusionArcReadings: {arcReadings}")
        start_angle = arc[0]/540 * 2*np.pi
        end_angle = arc[1]/540 * 2*np.pi
        occlusion_angles = np.linspace(start_angle,end_angle, len(arcReadings))
        if arc == realOcclusionArcs[0]:
            ax.plot(occlusion_angles, arcReadings, 'y-', linewidth=3, label='Occlusion')
        else:
            ax.plot(occlusion_angles, arcReadings, 'y-', linewidth=3) 

    print(f'freeArcsComponent: {freeArcsComponent}, occlusionComponent: {occlusionComponent}')
    plt.quiver(0, 0, freeArcsComponent[0], freeArcsComponent[1], color='b', label='Free Arcs Force')
    plt.quiver(0, 0, occlusionComponent[0], occlusionComponent[1], color='y', label='Occlusion Force')
    plt.quiver(0, 0, occlusionComponent[0]+freeArcsComponent[0], occlusionComponent[1]+freeArcsComponent[1], color='k', label='Resultant Force')
    # Plot drone marker at center
    ax.plot(0, 0, 'k^', markersize=15, label='Drone')    
    
    ax.set_title('LIDAR Scan')
    ax.set_theta_zero_location('N')  # 0 degrees at North
    ax.set_theta_direction(-1)  # clockwise
    ax.grid(True)
    ax.legend()
    
    plt.show()

if __name__ == '__main__':
    readings = read_lidar_data()
    freeArcs,realOcclusionArcs = process_lidar_data(readings)
    freeArcsComponent,occlusionComponent = control_law(readings,freeArcs,realOcclusionArcs)
    plot_lidar_data(readings,freeArcs,realOcclusionArcs,freeArcsComponent,occlusionComponent)
