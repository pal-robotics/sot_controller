'''
@author: Karsten Knese, Gennaro Raiola
'''
from sot_ros_api import *
from sot_tasks.startup_tasks import *

from dynamic_graph.dynamic_graph_fcl import DynamicGraphFCL
from sot_tasks.meta_task_dynamic_velocity_damping import MetaTaskDynamicVelocityDamping
from dynamic_graph.sot.core.dyn_oppoint_modifier import DynamicOppointModifier

def followTrajectory():
	createRosExport('matrixHomoStamped',taskRW.featureDes.position,'/hand_pose_publisher/right_hand_ref_pose')
	taskRW.feature.selec.value = 111

def followMarker():
	createRosExport('vector3Stamped',taskHANDGAZE.proj.point3D,'/ball_grasping/ball_point')

def speedUp(gainvalue):
    taskRW.task.controlGain.value = gainvalue

def grasp():
    pop(taskRW_safe)
    push(taskGRASP)

def resetRW():
    pop(taskGRASP)
    push(taskRW_safe)

# Create basic tasks
taskRW = createEqualityTask('rightWrist', 'hand_right_sot_grasping_frame_joint')

taskRW_safe = createEqualityTask('rightWrist_safe', 'arm_right_tool_joint')
quat = numpy.array([0.0,0.7071067811865476,0.0,0.7071067811865476])
xyz = numpy.array([-0.1,-0.3,1.0])
rw_safe_goal =  goalDef(xyz, quat)
gotoNd(taskRW_safe, rw_safe_goal, '111111',10)


diag = (0,0,0,0,0,0,1.36499,1.49997,0.677897,0.636507,0.170576,0.234007,0.120986,0.0156722,0.0213592,0.687228,0.63189,0.172151,0.260947,0.120986,0.0156722,0.0213592,0.511255,0.520094)
taskWEIGHTS = createWeightsTask(diag)

taskGRASP = createGraspingTask('hand_right_sot_grasping_frame_joint')
taskGRASP.goto3D((0.4,0.0,1.3),1)

basicStack()
gotoNd(taskRW,(0.4,0.0,1.3),'111',1)
taskGRASP.task.add(taskRW.feature.name)
#push(taskRW)
push(taskGRASP)

#followMarker()
