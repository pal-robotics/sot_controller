'''
@author: Gennaro Raiola, Karsten Knese
'''
from sot_ros_api import *

def initGaze():
    pop(taskSAFEPOS)
    push(taskGAZE)
    push(taskWEIGHTS)
    taskGAZE.goto3D((100,0.0,1.48),10)

def startGaze():
    createRosExport('vector3Stamped',taskGAZE.proj.point3D,'move_head_towards_largest_circle_stereo/ball_position_vect_filt')

taskRW = createEqualityTask('rightWrist', 'arm_right_tool_joint',1)
taskBASE = createEqualityTask('baseContact','base_joint',10)
taskJL = createJointLimitsTask(200)
taskGAZE = createGazeTask('camera_joint')
taskSAFEPOS = safePosition()

weights_diag_flag = 1
if (weights_diag_flag):
    diag = (0,0,0,0,0,0,1.36499,1.49997,0.677897,0.636507,0.170576,0.234007,0.120986,0.0156722,0.0213592,0.687228,0.63189,0.172151,0.260947,0.120986,0.0156722,0.0213592,0.511255,0.520094)
else:
    diag = None

taskWEIGHTS = createWeightsTask(diag)

push(taskJL)
solver.addContact(taskBASE)
push(taskSAFEPOS)

#createRosExport('matrixHomoStamped',taskRW.featureDes.position,'move_head_towards_largest_circle_stereo/ball_position_filt')
