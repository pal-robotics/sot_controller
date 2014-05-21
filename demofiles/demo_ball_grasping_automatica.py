'''
@author: Karsten Knese, Gennaro Raiola
'''
from sot_ros_api import *
from sot_tasks.startup_tasks import *

from dynamic_graph.dynamic_graph_fcl import DynamicGraphFCL
from sot_tasks.meta_task_dynamic_velocity_damping import MetaTaskDynamicVelocityDamping
from dynamic_graph.sot.core.dyn_oppoint_modifier import DynamicOppointModifier

import time

def followTrajectory():
	createRosExport('matrixHomoStamped',taskGRASP.featureDes.position,'/move_head_towards_largest_circle_stereo/ball_transform')

def followBall():
	createRosExport('vector3Stamped',taskGAZE.proj.point3D,'/move_head_towards_largest_circle_stereo/ball_vector3')

def speedUp(gainvalue):
    taskRW.task.controlGain.value = gainvalue

def grasp():
    solver.clear()
    basicStack()
    push(taskGAZE)
    push(taskGRASP)
    push(taskWEIGHTS)

def resetRW():
    solver.clear()
    basicStack()
    push(taskGAZE)
    push(taskRW_safe)
    push(taskWEIGHTS)

def resetJoints():
    solver.clear()
    basicStack()
    gotoSafePos(5,0.05)
    push(taskGAZE)


# Create basic tasks
taskRW_safe = createEqualityTask('rightWrist_safe', 'arm_right_tool_joint')
quat = numpy.array([0.0,0.7071067811865476,0.0,0.7071067811865476])
xyz = numpy.array([-0.1,-0.3,1.0])
rw_safe_goal =  goalDef(xyz, quat)
gotoNd(taskRW_safe, rw_safe_goal, '111111',10)

reset_joints = (0,0,0,0,0,0,-0.4, 0.4, -0.1, 0.6109, 0, 0, 0, -0.4, 0.4, -0.1, 0.6109, 0, 0, 0, 0.0, 0.0, 0.0, 0.0)
taskJOINTS_safe = createWeightsTask(reset_joints)

taskGAZE = createGazeTask('camera_joint')
taskGAZE.goto3D((100,0.0,1.48),10)

diag = (0,0,0,0,0,0,1.36499,1.49997,0.677897,0.636507,0.170576,0.234007,0.120986,0.0156722,0.0213592,0.687228,0.63189,0.172151,0.260947,0.120986,0.0156722,0.0213592,0.511255,0.520094)
taskWEIGHTS = createWeightsTask(diag)

taskGRASP = createEqualityTask('hand_right_sot_grasping_frame_joint', 'hand_right_sot_grasping_frame_joint')
gotoNd(taskGRASP, (0.3,-0.3,1.3),'111111',10)
'''
basicStack()
push(taskGAZE)
push(taskGRASP)
push(taskWEIGHTS)'''
