import time
import numpy as np
from controller import Supervisor
import sys
import os
# adding Folder_2 to the system path
sys.path.insert(0, os.path.expanduser('~/Documents/flush/scripts'))
print(sys.path)
#sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from RLVBVP3 import RLVBVP3

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance
robotino_node = robot.getFromDef('ROBOT0')
translation_field = robotino_node.getField('translation')
position = translation_field.getSFVec3f()
id = robotino_node.getField('name').getSFString()

lidar=robot.getDevice('lidar')
lidar_node = robot.getFromDef('LIDAR')

reading_radius = lidar.getMaxRange()
lidar.enable(TIME_STEP)
#lidar_node.getField('maxRange').setSFloat(2.0)
lidar.enablePointCloud()

# Get emitter and receiver
emitter = robot.getDevice('emitter')
receiver = robot.getDevice('receiver')
receiver.enable(TIME_STEP)

robot_control = RLVBVP3(pos=position[:-1],comms_radius=reading_radius*2,
                        reading_radius=reading_radius,resolution=540,id=id,
                        )

while robot.step(TIME_STEP) != -1:
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

  