'''
This is a basic demo with reem or reemc robot using the stack of tasks.
The stack is composed by:
    - joints limits constraint
    - base link contact (this constraint fixes the robot free flyer)
    - reach-task with right wrist

@author: Gennaro Raiola, Karsten Knese
'''
from sot_ros_api import *
from sot_tasks.startup_tasks import *

def resetPosition():
    pop(taskRW)
    push(taskRESET)
'''necessary for setting up the basic stack
1.) joint limits
2.) base contact (setting free flyer)
3.) self-collision activated (boolean flag)
'''
basicStack()

#quat = numpy.array([-0.377,-0.06,-0.142,0.91])
#xyz = numpy.array([0.35,-0.3,1.25])
#goal_rw = goalDef(xyz,quat)

taskRW = createEqualityTask('rightWrist', 'hand_right_sot_grasping_frame_joint')
#taskRESET = createJointsTask()

push(taskRW)
gotoNd(taskRW,(0.3,-0.3,1.2),'111111',10)

getEntFCL().enableCapsuleTopic(True)
createRosImport('matrixHomo',taskSC.task.signal('p1_arm_right_7_jointtorso_1_joint') , '/sot_controller/p1_arm_right_7_jointtorso_1_joint')
createRosImport('matrixHomo',taskSC.task.signal('p2_arm_right_7_jointtorso_1_joint') , '/sot_controller/p2_arm_right_7_jointtorso_1_joint')


