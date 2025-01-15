import math
import matplotlib.pyplot as plt
import numpy as np
import os
from lidar_processing_points import * 

def plot_lidar_data(readings,freePointIdx,occlusionVectorPoints,freeArcsComponent,occlusionComponent):
    angles = np.linspace(0.5*np.pi, -1.5*np.pi, len(readings))
    free_x = [readings[p] * np.cos(angles[p]) for p in freePointIdx]
    free_y = [readings[p] * np.sin(angles[p]) for p in freePointIdx]
    readings_x = [readings[p] * np.cos(angles[p]) for p in range(len(readings))]
    readings_y = [readings[p] * np.sin(angles[p]) for p in range(len(readings))]
    occlusion_x = [readings[p[0]] * np.cos(angles[p[0]]) for p in occlusionVectorPoints]
    occlusion_y = [readings[p[1]] * np.sin(angles[p[1]]) for p in occlusionVectorPoints]
    plt.figure(figsize=(10, 10))
    plt.quiver(0, 0, freeArcsComponent[0], freeArcsComponent[1], color='green', angles='xy', scale_units='xy', scale=1, label='Free Component')
    plt.quiver(0, 0, occlusionComponent[0], occlusionComponent[1], color='red', angles='xy', scale_units='xy', scale=1, label='Occlusion Component')
    plt.scatter(readings_x, readings_y, c='blue', label='Obstacle Points')
    plt.scatter(free_x, free_y, c='green', label='Free Points')
    plt.plot(occlusion_x, occlusion_y, 'r-', label='Occlusion Line')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.title('Lidar Data Visualization')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()
    



if __name__ == '__main__':
    readings = read_lidar_data()
    freePointIdx,occlusionVectorPoints = process_lidar_data(readings)
    freeArcsComponent,occlusionComponent = control_law(readings,freePointIdx,occlusionVectorPoints)
    plot_lidar_data(readings,freePointIdx,occlusionVectorPoints,freeArcsComponent,occlusionComponent)