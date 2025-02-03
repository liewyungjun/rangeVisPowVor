import time
import sys
import os
import math
import shapely
from shapely import MultiPoint
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union,split
from shapely.geometry import LineString
import logging
import matplotlib.pyplot as plt
import numpy as np
print(sys.path)
#sys.path.insert(0, os.path.expanduser('~/Documents/flush/scripts'))
#from RLVBVP3 import RLVBVP3   

def read_coords(filepath):
    try:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        full_path = os.path.join(current_dir, filepath)
        
        with open(full_path, 'r') as file:
            content = file.read()
            # Remove brackets and split into rows
            content = content.strip('[]')
            rows = content.split('],')
            
            coords = []
            for row in rows:
                # Clean up the string and split into values
                row = row.strip('[] \n')
                values = row.split()
                try:
                    coords.append([float(values[0]), float(values[1])])
                except IndexError:
                    #time.sleep(0.01)
                    continue
                
        return coords
    except FileNotFoundError:
        logging.error(f"Error: File {filepath} not found at {full_path}")
        return None

    

if __name__ == "__main__":
    progress = []
    timestep = 1.0
    while True:
        try:
            polys = []
            for i in range(10):
                coords = read_coords(f'webots_coords_{i}.txt')
                if coords:
                    poly = Polygon(coords)
                    polys.append(poly)
            for j in range(2):
                totalPoly = unary_union(polys)
            progress.append(totalPoly.area)
            time.sleep(timestep)
        except KeyboardInterrupt:
            break
    plt.clf()
    plt.plot(range(len(progress)), progress, 'b-')
    plt.xlabel('Timestep')
    plt.ylabel('Area')
    plt.title('Progress over Time')
    plt.grid(True)
    plt.show()
    #plt.pause(0.01)
    