from controller import Supervisor



TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance
robotino_node = robot.getFromDef('ROBOTINO')
translation_field = robotino_node.getField('translation')
position = translation_field.getSFVec3f()
velocity = [0.0,0.0,0.0]
print()
# [CODE PLACEHOLDER 1]

i = 0
while robot.step(TIME_STEP) != -1:
  # [CODE PLACEHOLDER 2]
  if i == 0:
      new_value = [2.5, 0, 0]
#      translation_field.setSFVec3f(new_value)
  i += 1
