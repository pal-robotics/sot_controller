'''
@author: Karsten Knese
Import notes to consider:
because of some instability of the solver in terms of numerial errors
it's highly recommended by our side to stick to the following gains:

- joints limits have to be not less than 500
- a full positioning in terms of Kinematic Meta tasks have to be provided: '111 111'
- the task gain can be variable but a working solution is a simple gain of 1

'''
from sot_ros_api import *

taskJL = createJointLimitsTask(500)
push(taskJL)

taskBASE = createEqualityTask('baseContact','base_joint',1000)
solver.addContact(taskBASE)

taskRW = createEqualityTask('rightWrist', 'arm_right_tool_joint')
gotoNd(taskRW, (-0.1,-0.4,1.1),'111111',1)

goalP2 = goalDef((0.3,0.0, 1.1))
taskDAMP = createVelocityDampingTask('velDamp', 'arm_right_tool_joint', goalP2, 0.4, 0.2)
taskDAMP.task.controlGain.value = 1
createRosImport('matrixHomoStamped', taskDAMP.task.p1, 'sot_rviz_marker/position1')
createRosImport('matrixHomoStamped', taskDAMP.task.p2, 'sot_rviz_marker/position2')
createRosImport('vector3Stamped', taskDAMP.task.n, 'sot_rviz_marker/unitvec')
createRosImport('double', taskDAMP.task.ds, 'sot_rviz_marker/ds')
createRosImport('double', taskDAMP.task.di, 'sot_rviz_marker/di')

push(taskDAMP)
push(taskRW)