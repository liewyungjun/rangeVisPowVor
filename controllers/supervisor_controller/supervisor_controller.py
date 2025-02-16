import time
import numpy as np
from controller import Supervisor
import sys
import os
# adding Folder_2 to the system path
sys.path.insert(0, os.path.expanduser('~/Documents/flush/scripts'))
print(sys.path)
#sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from RLVBVP4 import RLVBVP4

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance
robotino_node = robot.getFromDef('ROBOT0')
translation_field = robotino_node.getField('translation')
position = translation_field.getSFVec3f()
id = robotino_node.getField('name').getSFString()

lidar=robot.getDevice('lidar')
lidar_node = robot.getFromDef('LIDAR')

grid_node = robot.getFromDef('ROBOT')
grid_centroid = grid_node.getField('translation').getSFVec3f()
old_gc = grid_centroid
grid_dim = np.array([12.0,12.0])

reading_radius = lidar.getMaxRange()
lidar.enable(TIME_STEP)
#lidar_node.getField('maxRange').setSFloat(2.0)
lidar.enablePointCloud()

# Get emitter and receiver
emitter = robot.getDevice('emitter')
receiver = robot.getDevice('receiver')
receiver.enable(TIME_STEP)

robot_control = RLVBVP4(pos=position[:-1],comms_radius=reading_radius*2,
                        reading_radius=reading_radius,resolution=540,id=id,
                        grid_centroid=np.array([grid_centroid[:2]]),grid_dim=grid_dim)

while robot.step(TIME_STEP) != -1:
  robot_control.grid_centroid = grid_node.getField('translation').getSFVec3f()[:2]
  if abs(sum(old_gc) - sum(robot_control.grid_centroid)) > 0.1:
    print(f'{robot_control.id}: CHANGED GC to {robot_control.grid_centroid}')
    old_gc = robot_control.grid_centroid[:]
  range_image=lidar.getRangeImage()

  neighbourArr = [] # Receive messages
  while receiver.getQueueLength() > 0:
    message = receiver.getString()
    coords = [float(x.strip()) for x in message.strip('[]').split(',')]
    neighbourArr.append(coords)
    receiver.nextPacket()
  if not neighbourArr:
    neighbourArr = [[]]

  robot_control.step(neighbourArr,range_image,TIME_STEP)
  webots_position = translation_field.getSFVec3f()
  new_pos = [robot_control.pos[0],robot_control.pos[1],webots_position[-1]]
  translation_field.setSFVec3f(new_pos)

  message = f"{new_pos}"
  emitter.send(message.encode('utf-8'))

  # if robot_control.id == '0':
  #   print(f'{robot_control.id}:----------------')
  #   print(f'Neighbours: {robot_control.neighbour_coords}')
  #   print(f'Filtered free points:{len(robot_control.filteredFreePointCoords)}')
  #   print(f'Position: {robot_control.pos}')
  #   print(f'Free arcs: {robot_control.freeArcsComponent}')
  #   print(f'Occ arcs: {robot_control.occlusionArcsComponent}')
  #   print(f'Velocity: {robot_control.velocity}')
  #   print(f'Grid_Centroid: {robot_control.grid_centroid}')
    