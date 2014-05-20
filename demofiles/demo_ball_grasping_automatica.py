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
	createRosExport('matrixHomoStamped',taskRW.featureDes.position,'/hand_pose_publisher/right_hand_ref_pose')
	taskRW.feature.selec.value = 111

def followBall():
	createRosExport('vector3Stamped',taskGAZE.proj.point3D,'/ball_grasping/ball_point')

def speedUp(gainvalue):
    taskRW.task.controlGain.value = gainvalue

def grasp():
    pop(taskWEIGHTS)
    pop(taskRW_safe)
    push(taskGRASP)
    pop(taskWEIGHTS)

def resetRW():
    pop(taskWEIGHTS)
    pop(taskGRASP)
    push(taskRW_safe)
    push(taskWEIGHTS)

def resetJoints():
    solver.clear()
    gotoSafePos(3,0.1)
    basicStack()


# Create basic tasks
taskRW = createEqualityTask('rightWrist', 'hand_right_sot_grasping_frame_joint')

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

basicStack()
push(taskGAZE)
push(taskGRASP)
push(taskWEIGHTS)
