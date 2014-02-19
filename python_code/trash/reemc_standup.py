#!/usr/bin/env python
from sot_ros_api import *

def startTorso():
    goal_lw = goalDef([0.1,0.0,0.95],[0.0,0.0,0.0,1.0])
    gotoNdComp(taskTORSO,goal_lw,'000110',10)

def startHead():
    goal_lw = goalDef([0.0,0.0,1.28],[0.0,0.0,0.0,1.0])
    gotoNdComp(taskHEAD,goal_lw,'111111',10)

def start():
    push(taskWEIGHTS)

taskJL = createJointLimitsTask(gain = 250)
plug(robot.device.dt,taskJL.dt)

taskRS = createEqualityTask('rightSole', 'right_sole_joint',10)
taskLS = createEqualityTask('leftSole','left_sole_joint',10)
taskTORSO = createEqualityTask('torso', 'torso_2_joint')
taskHEAD = createEqualityTask('head', 'head_1_joint')

weights_diag_flag = 1
if (weights_diag_flag):
    diag = numpy.ones(36)
    diag = tuple(diag)
else:
    diag = None

taskWEIGHTS = createWeightsTask(diag,gain = 1)
ref = numpy.zeros(36)
taskWEIGHTS.task.setVelocity(tuple(ref))
taskWEIGHTS.task.setPositionDes(tuple(ref))
plug(robot.dynamic.position,taskWEIGHTS.task.position)

offset_x = 0.0
offset_y = 0.0

robot.dynamic.com.recompute(0)
createRosImport('vector3',robot.dynamic.com, 'sot_controller/com_position')
taskCOM = createComIneqTask(gain = 1,dt = 1, referenceInf = (0.0 + offset_x,-0.005 + offset_y, 0), referenceSup = (0.005 + offset_x,0.005 + offset_y,0))
plug(robot.device.dt,taskCOM.dt)

push(taskJL)
solver.addContact(taskRS)
solver.addContact(taskLS)