#!/usr/bin/python
from sot_ros_api import *

taskLW = MetaTaskKine6d('lw',robot.dynamic,'arm_left_tool_joint','arm_left_tool_joint')
taskLW.feature.frame('desired')

robot.dynamic.upperJl.recompute(0)
robot.dynamic.lowerJl.recompute(0)
taskJL = TaskJointLimits('taskJL')
plug(robot.dynamic.position,taskJL.position)
taskJL.controlGain.value = 1
taskJL.referenceInf.value = robot.dynamic.lowerJl.value
taskJL.referenceSup.value = robot.dynamic.upperJl.value
taskJL.dt.value = 1
taskJL.selec.value = toFlags(range(6,robot.dimension))

taskRL = MetaTaskKine6d('contact_rl',robot.dynamic,'leg_right_6_joint','leg_right_6_joint')
taskRL.feature.frame('desired')
taskRL.gain.setConstant(1)

taskLL = MetaTaskKine6d('contact_ll',robot.dynamic,'leg_left_6_joint','leg_left_6_joint')
taskLL.feature.frame('desired')
taskLL.gain.setConstant(1)

taskCOM = createComIneqTask(dt = 1, referenceInf = (-0.1,-0.1, 0), referenceSup = (0.1,0.1,0))

push(taskJL)
push(taskCOM)
solver.addContact(taskRL)
solver.addContact(taskLL)
push(taskLW)

gotoNd(taskLW,((0.2,0.4,0.5)),'111',1)
