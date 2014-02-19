'''
In this demo non-actuated joints are used to achieve two tasks: reach a point in the operational space and align the gaze
with this point.
The stack is composed by:
    - joints limits constraint
    - base link contact (this constraint fixes the robot free flyer)
    - reach-task with right wrist
    - gaze-task
    
Important note:
Jrl-dynamics uses a different convention for the actuated joint frames, this is why
gotoNdComp and goalDefComp are provided. For fixed joints that respect the frame convention used in
jrl-dynamics is possible to use the standard functions such as gotoNd and the goto3D. This is the case with
camera_joint and arm_right_tool_joint. 

@author: Gennaro Raiola, Karsten Knese
'''
from sot_ros_api import *

def resetPosition():
    pop(taskGAZE)
    pop(taskRW)
    push(taskRESET)

quat = numpy.array([-0.377,-0.06,-0.142,0.91])
xyz = numpy.array([0.35,-0.3,1.25])

goal_rw = goalDef(xyz,quat)

taskJL = createJointLimitsTask(100)
taskBASE = createEqualityTask('baseContact','base_joint',10)
taskRW = createEqualityTask('rightWrist', 'arm_right_tool_joint')
taskGAZE = createGazeTask('camera_joint')
taskRESET = createJointsTask()

push(taskJL)
solver.addContact(taskBASE)
push(taskRW)
push(taskGAZE)

gotoNd(taskRW,goal_rw,'111111',1)
taskGAZE.goto3D((xyz[0],xyz[1],xyz[2]),1)
