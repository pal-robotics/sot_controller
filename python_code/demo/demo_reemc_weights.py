#!/usr/bin/env python
from sot_ros_api import *

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

ref = numpy.zeros(36)
refVel = numpy.zeros(36)

# Arm left
ref[20] = -0.49552838669100613;
ref[21] = 0.024631160165058508;
ref[22] = 0.3587618799256007;
ref[23] = 1.7700407547314527;
ref[24] = -0.3221388264942749;
ref[25] = -0.12734436348362826;
ref[26] = 0.00845979656312204;
# Arm right
ref[27] = -0.1320161376919419;
ref[28] = 0.3812710840786022;
ref[29] = 0.10791338229372394;
ref[30] = 1.3847557170524056;
ref[31] = 0.12271955295019997;
ref[32] = -0.11658357530268998;
ref[33] = 0.07976770941762998;
# Head
ref[34] = -0.0007797819167295179;
ref[35] = -6.391655055159984e-05;
# Leg left
ref[6] = 0.00019271088363695223;
ref[7] = -0.012281374195991613;
ref[8]= -1.5539225381281545;
ref[9] = 2.0989627142381795;
ref[10] = -0.4325825821837508;
ref[11] = -0.0076794318323346895;
# Leg right
ref[12] = -0.0994575641321298;
ref[13] = 0.038330463911328064;
ref[14] = -1.572524176478941;
ref[15] = 2.100818017809862;
ref[16] = -0.42492891095648416;
ref[17] = -0.07516505860639641;
# Torso
ref[18] = -0.0440104925677645;
ref[19] = 0.05530074092254829;

taskWEIGHTS = createWeightsTask(diag,gain = 1)
taskWEIGHTS.task.setVelocity(tuple(refVel))
taskWEIGHTS.task.setPositionDes(tuple(ref))
plug(robot.dynamic.position,taskWEIGHTS.task.position)

offset_x = 0.0
offset_y = 0.0

robot.dynamic.com.recompute(0)
taskCOM = createComIneqTask(gain = 1,dt = 1, referenceInf = (-0.1 + offset_x,-0.005 + offset_y, 0), referenceSup = (0.005 + offset_x,0.005 + offset_y,0))
plug(robot.device.dt,taskCOM.dt)

push(taskJL)
push(taskCOM)
solver.addContact(taskRS)
solver.addContact(taskLS)
