from sot_ros_api import *

xyz = numpy.array([0,-100,1.483])

taskJL = createJointLimitsTask(100)
taskBASE = createEqualityTask('baseContact','base_joint',10)
taskHEAD = createGazeTask('camera_joint')

push(taskJL)
solver.addContact(taskBASE)
push(taskHEAD)

taskHEAD.goto3D((xyz[0],xyz[1],xyz[2]),10)
