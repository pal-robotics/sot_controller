'''
@author: Karsten Knese, Gennaro Raiola
'''
from sot_ros_api import *
from sot_tasks.startup_tasks import *

from dynamic_graph.dynamic_graph_fcl import DynamicGraphFCL
from sot_tasks.meta_task_dynamic_velocity_damping import MetaTaskDynamicVelocityDamping
from dynamic_graph.sot.core.dyn_oppoint_modifier import DynamicOppointModifier

def followTrajectory():
    createRosExport('matrixHomoStamped',taskGRASP.featureDes.position,'/move_head_towards_largest_circle_stereo/ball_transform')
    createRosExport('vector3Stamped',taskALIGN.proj.point3D,'/move_head_towards_largest_circle_stereo/ball_vector3')
    taskGRASP.feature.selec.value = '100111'
    taskGRASP.task.controlGain.value = 0.5
    taskALIGN.task.controlGain.value = 20

def followMarker():
    createRosExport('vector3Stamped',taskCAMERA.proj.point3D,'/move_head_towards_largest_circle_stereo/ball_vector3')

def graspPosition(pos=(0.3,-0.1,1.3), gotoGain=1, lookGain=10):
    gotoNd(taskGRASP,pos,'100111',gotoGain)
    taskLOOK.goto3D((0.3,-0.1,1.3),lookGain)
    pop(taskRW_safe)
    push(taskALIGN)
    push(taskGRASP)

def graspBall():
    pop(taskRW_safe)
    push(taskALIGN)
    push(taskGRASP)

def resetRW():
    pop(taskGRASP)
    pop(taskALIGN)
    push(taskRW_safe)

# Create basic tasks
taskRW_safe = createEqualityTask('rightWrist_safe', 'arm_right_tool_joint')
#quat = numpy.array([0.0,0,0.0,0.7071067811865476])
#xyz = numpy.array([-0.1,-0.3,1.0])
#rw_safe_goal =  goalDef(xyz, quat)
#gotoNd(taskRW_safe, rw_safe_goal, '111111',10)
gotoNd(taskRW_safe, (-0.1,-0.3,1.1),'111111',10)

diag = (0,0,0,0,0,0,1.36499,1.49997,0.677897,0.636507,0.170576,0.234007,0.120986,0.0156722,0.0213592,0.687228,0.63189,0.172151,0.260947,0.120986,0.0156722,0.0213592,0.511255,0.520094)
taskWEIGHTS = createWeightsTask(diag)

taskCAMERA = createGazeTask('camera_joint')
taskCAMERA.goto3D((100,0.0,1.48),10)

grasping_joint = 'hand_right_sot_grasping_frame_joint'
#taskLOOK = createGazeTask(grasping_joint)
taskALIGN = createGraspingTask(grasping_joint)
taskALIGN.goto3D((0.3,-0.3,1.3),20)
taskGRASP = createEqualityTask(grasping_joint, grasping_joint)
gotoNd(taskGRASP, (0.3,-0.3,1.3),'111',1)

basicStack()
push(taskCAMERA)
createRosImport('matrixHomoStamped', taskGRASP.featureDes.position, '/sot_controller/grasp_goal_transform')

