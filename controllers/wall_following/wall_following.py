import time
import numpy as np
from controller import Supervisor
import sys
import os
# adding Folder_2 to the system path
sys.path.insert(0, os.path.expanduser('~/Documents/flush/scripts'))
print(sys.path)
#sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from WF import WF

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance
robotino_node = robot.getFromDef('ROBOT0')
translation_field = robotino_node.getField('translation')
rotation_field = robotino_node.getField('rotation')
position = translation_field.getSFVec3f()
id = robotino_node.getField('name').getSFString()

lidar=robot.getDevice('lidar')
lidar_node = robot.getFromDef('LIDAR')

reading_radius = lidar.getMaxRange()
lidar.enable(TIME_STEP)
#lidar_node.getField('maxRange').setSFloat(2.0)
lidar.enablePointCloud()

robot_control = WF(pos=position[:-1],reading_radius=reading_radius,resolution=540,id=id)

while robot.step(TIME_STEP) != -1:
  range_image=lidar.getRangeImage()
  #print(range_image)

  robot_control.step(range_image=range_image,timestep=TIME_STEP)
  webots_position = translation_field.getSFVec3f()
  webots_rotation = rotation_field.getSFRotation()
  new_pos = [robot_control.pos[0],robot_control.pos[1],webots_position[-1]]
  new_rot = [webots_rotation[0],webots_rotation[1],webots_rotation[2],robot_control.theta]
  translation_field.setSFVec3f(new_pos)
  rotation_field.setSFRotation(new_rot)
