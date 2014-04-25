'''
@author: Karsten Knese, Gennaro Raiola
'''
from sot_ros_api import *

from dynamic_graph.dynamic_graph_fcl import DynamicGraphFCL
from sot_tasks.meta_task_dynamic_velocity_damping import MetaTaskDynamicVelocityDamping
from dynamic_graph.sot.core.dyn_oppoint_modifier import DynamicOppointModifier

def followTrajectory():
	createRosExport('matrixHomoStamped',taskRW.featureDes.position,'/hand_pose_publisher/right_hand_ref_pose')

def followMarker():
	createRosExport('vector3Stamped',taskGAZE.proj.point3D,'/aruco_single/position')

def gotoMarker():
	createRosExport('matrixHomoStamped',taskRW.featureDes.position,'/aruco_single/pose')

def speedUp(gainvalue):
    taskRW.task.controlGain.value = gainvalue

def avoidObject():
	createRosExport('matrixHomoStamped',taskDAMP.task.signal('p2_'+actuated_joint+'external'),'/hand_pose_publisher/left_hand_ref_pose')

	createRosImport('double', taskDAMP.task.signal('d_'+actuated_joint+'external'), '/sot_controller/d_'+actuated_joint+'external')
	createRosImport('matrixHomoStamped',taskDAMP.task.signal('p1_'+actuated_joint+'external') , '/sot_controller/p1_'+actuated_joint+'external')
	createRosImport('matrixHomoStamped',taskDAMP.task.signal('p2_'+actuated_joint+'external') , '/sot_controller/p2_'+actuated_joint+'external')
	createRosImport('double',taskDAMP.task.signal('ds_'+actuated_joint+'external') , '/sot_controller/ds_'+actuated_joint+'external')
	createRosImport('double',taskDAMP.task.signal('di_'+actuated_joint+'external') , '/sot_controller/di_'+actuated_joint+'external')

def exposeWristPosition():
	createRosImport('matrixHomoStamped', taskRW.feature.position, '/sot_controller/taskRW_feature')
	createRosImport('matrixHomoStamped', taskRW.featureDes.position, '/sot_controller/taskRW_featuredes')

def basicStack():
	# Basic stack
	push(taskJL)
	solver.addContact(taskBASE)

actuated_joint = 'arm_right_tool_joint'
robot.dynamic.createOpPoint(actuated_joint, actuated_joint)
taskDAMP = MetaTaskDynamicVelocityDamping('selfcollision')
taskDAMP.task.setAvoidingObjectPair(actuated_joint, 'external')
#create all the signals
taskDAMP.task.finalizeSignals()
taskDAMP.task.controlGain.value = 1

# plug tool position into p1 and the respective moving jacobian
# p2 will not be set, hence it might print p2-ptr not set. 
# This will be set by activating the rosexport 
plug(robot.dynamic.signal(actuated_joint), taskDAMP.task.signal('p1_'+actuated_joint+'external'))
plug(robot.dynamic.signal('J'+actuated_joint), taskDAMP.task.signal('jVel_'+actuated_joint+'external') )
taskDAMP.task.signal('ds_'+actuated_joint+'external').value = 0.1
taskDAMP.task.signal('di_'+actuated_joint+'external').value = 0.3

# Create basic tasks
taskRW = createEqualityTask('rightWrist', 'arm_right_tool_joint')
taskBASE = createEqualityTask('baseContact','base_joint',10)
taskJL = createJointLimitsTask(200)
safePos = safePosition()
diag = (0,0,0,0,0,0,1.36499,1.49997,0.677897,0.636507,0.170576,0.234007,0.120986,0.0156722,0.0213592,0.687228,0.63189,0.172151,0.260947,0.120986,0.0156722,0.0213592,0.511255,0.520094)
taskWEIGHTS = createWeightsTask(diag)

basicStack()
gotoNd(taskRW,(0.2,-0.5,1.1),'111111',5)

#followTrajectory()
#avoidObject()
push(taskDAMP)
push(taskRW)
#push(taskWEIGHTS)

exposeWristPosition()
