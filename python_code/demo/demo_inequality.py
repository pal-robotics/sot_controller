from sot_ros_api import *

taskBASE = createEqualityTask('baseContact','base_joint',1000)
taskJL = createJointLimitsTask(300)
taskIneq = createInequalityTask('rightWrist', 'arm_right_tool_joint', '000111', (0,0,0), (-0.1,-0.4,0.9), (0.4,0.0,1.1))
taskRW = createEqualityTask('rightWrist', 'arm_right_tool_joint')
gotoNd(taskRW, (0.2,-0.3,1.1), '111', 100)
push(taskJL)
solver.addContact(taskBASE)
push(taskIneq)
push(taskRW)
