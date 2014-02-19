'''
This is a basic demo with reem or reemc robot using the stack of tasks.
The stack is composed by:
    - joints limits constraint
    - base link contact (this constraint fixes the robot free flyer)
    - reach-task with right wrist

@author: Gennaro Raiola, Karsten Knese
'''
from sot_ros_api import *

def resetPosition():
    pop(taskRW)
    push(taskRESET)

quat = numpy.array([-0.377,-0.06,-0.142,0.91])
xyz = numpy.array([0.35,-0.3,1.25])

goal_rw = goalDef(xyz,quat)

taskJL = createJointLimitsTask(100)
taskBASE = createEqualityTask('baseContact','base_joint',10)
taskRW = createEqualityTask('rightWrist', 'arm_right_7_joint')
#taskRESET = createJointsTask()

push(taskJL)
solver.addContact(taskBASE)
push(taskRW)

gotoNdComp(taskRW,goal_rw,'000111',1)
