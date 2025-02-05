import numpy as np
import matplotlib.pyplot as plt

resolution = 120
radius = 3.0
fov = 140

def findY(fov):
    fov_in_radian = fov/360 * 2*np.pi
    angle_start = 0.5*np.pi + fov_in_radian/2
    angle_end = 0.5*np.pi - fov_in_radian/2
    angles = np.linspace(angle_start, angle_end, resolution)
    reading_coords = np.array([[radius*np.cos(angles[i]),
                                    radius*np.sin(angles[i])]
                                    for i in range(len(angles))])

    pointsPerUnitLength = resolution/(fov_in_radian * radius)
    arcForce = np.zeros(2)
    for i in reading_coords:
        arcForce += i

    fovForce = np.zeros(2)
    #print(np.cos((np.pi-fov_in_radian)/2))
    fovForce = 2 * radius * pointsPerUnitLength * np.cos((np.pi-fov_in_radian)/2) * 3
    #print(f'arcForce: {arcForce}')
    #print(f'fovForce: {fovForce}')
    return arcForce[1],fovForce

res = []
for i in range(1,180):
    res.append([findY(i)])
#print(res)
arc = [i[0][0] for i in res] 
fovv = [i[0][1] for i in res] 
fov_values = [i for i in range(1,180)]
plt.plot(fov_values, arc, 'r')
plt.plot(fov_values, fovv, 'b')
plt.show()
#y = Σ (r * cos(0.5 * π + (f / 360) * π / 2 + i * ((0.5 * π - (f / 360) * π / 2) - (0.5 * π + (f0 / 360) * π / 2)) / (s - 1))) {0<i<s-1}
