'''
@author: Karsten Knese, Gennaro Raiola
'''
from sot_ros_api import *

from dynamic_graph.dynamic_graph_fcl import DynamicGraphFCL
from dynamic_graph.sot.core.meta_task_dynamic_velocity_damping import MetaTaskDynamicVelocityDamping
from dynamic_graph.sot.core.dyn_oppoint_modifier import DynamicOppointModifier

def followTrajectory():
	createRosExport('matrixHomoStamped',taskRW.featureDes.position,'/sot_filters/right_wrist_pose')
	createRosExport('matrixHomoStamped',taskLW.featureDes.position,'/sot_filters/left_wrist_pose')

def followMarker():
	createRosExport('vector3Stamped',taskGAZE.proj.point3D,'/aruco_single/position')

def gotoMarker():
	createRosExport('matrixHomoStamped',taskRW.featureDes.position,'/aruco_single/pose')

def speedUp(gainvalue):
    taskRW.task.controlGain.value = gainvalue
    taskLW.task.controlGain.value = gainvalue

def basicStack():
    # Basic stack
    push(taskJL)
    solver.addContact(taskBASE)
    
collision_objects = ['arm_right_3_joint', 'arm_right_7_joint', 'arm_right_5_joint','torso_1_joint','arm_left_3_joint', 'arm_left_7_joint', 'arm_left_5_joint', 'torso_2_joint', 'head_1_joint']
entFCL = DynamicGraphFCL('entFCL')

# setup entity for FCL
collision_string = ''
for collision in collision_objects:
	collision_string += collision+str(':')
collision_string = collision_string[:-1]
print ('setting collision objects: '+str(collision_string))
entFCL.set_collision_joints(collision_string)

# plugging FK into FCL 
for collision in collision_objects:
        robot.dynamic.createOpPoint(collision, collision)
	robot.dynamic.signal(collision).recompute(0)
	plug(robot.dynamic.signal(collision), entFCL.signal(collision))

for i in range(len(collision_objects)):
	for j in range(len(collision_objects)):
		if not(collision_objects[i] == collision_objects[j]):
			signal_name = str(collision_objects[i])+str(collision_objects[j])
			entFCL.signal(signal_name).recompute(0)
'''
TORSO TEST
DYNOP FOR CLOSEST POINT CALUCLATION AND JACOBIAN TWIST
'''

# just for torso test!
#arm right
arm_right_group = ['arm_right_3_joint', 'arm_right_7_joint', 'arm_right_5_joint']
arm_left_group = ['arm_left_3_joint', 'arm_left_7_joint', 'arm_left_5_joint']

dynOP_right_collection = []
dynOP_left_collection = []
'''
for arm_right_part in arm_right_group:
	dynOPright = DynamicOppointModifier(arm_right_part)
	plug(robot.dynamic.signal(arm_right_part), dynOPright.positionIN)
	plug(robot.dynamic.signal('J'+str(arm_right_part)), dynOPright.jacobianIN)
	plug(entFCL.signal('oppoint_'+str(arm_right_part)+'torso_1_joint'), dynOPright.transformationSig)
	dynOP_right_collection.append(dynOPright)




for arm_left_part in arm_left_group:
	dynOPleft = DynamicOppointModifier(arm_left_part)
	plug(robot.dynamic.signal(arm_left_part), dynOPleft.positionIN)
	plug(robot.dynamic.signal('J'+str(arm_left_part)), dynOPleft.jacobianIN)
	plug(entFCL.signal('oppoint_'+str(arm_left_part)+'torso_1_joint'), dynOPleft.transformationSig)
	dynOP_left_collection.append(dynOPleft)
'''
'''
AVOIDANCE TASK
'''
taskDAMP = MetaTaskDynamicVelocityDamping('velDamp', 0.1, 0.07)
taskDAMP.task.set_avoiding_objects(collision_string)
createRosImport('double', taskDAMP.task.ds, 'rviz_marker_closest_points/ds')
createRosImport('double', taskDAMP.task.di, 'rviz_marker_closest_points/di')


''' THIS HAS TO BE ENTFCL, otherwise the calculation won't get triggered!!!! IMPORTANT'''
''' question is why left arm is different in v compared to right
	
entFCL should return full pose of point, not just point with identity matrix!!

'''
#plug right arm
p2_signal = 'torso_1_joint'
for arm_right_part in arm_right_group:

	dynOPright = DynamicOppointModifier(arm_right_part+p2_signal)
	plug(robot.dynamic.signal(arm_right_part), dynOPright.positionIN)
	plug(robot.dynamic.signal('J'+str(arm_right_part)), dynOPright.jacobianIN)
	plug(entFCL.signal('oppoint_'+str(arm_right_part)+p2_signal), dynOPright.transformationSig)
	dynOP_right_collection.append(dynOPright)

	plug(entFCL.signal( str(arm_right_part)+p2_signal), taskDAMP.task.signal('p1_'+str(arm_right_part)+p2_signal ) )
	plug(entFCL.signal(p2_signal+str(arm_right_part)), taskDAMP.task.signal('p2_'+str(arm_right_part)+p2_signal))
	plug(dynOPright.signal('jacobian'), taskDAMP.task.signal('jVel_'+str(arm_right_part)+p2_signal))

p2_signal = 'torso_2_joint'
for arm_right_part in arm_right_group:

	dynOPright = DynamicOppointModifier(arm_right_part+p2_signal)
	plug(robot.dynamic.signal(arm_right_part), dynOPright.positionIN)
	plug(robot.dynamic.signal('J'+str(arm_right_part)), dynOPright.jacobianIN)
	plug(entFCL.signal('oppoint_'+str(arm_right_part)+p2_signal), dynOPright.transformationSig)
	dynOP_right_collection.append(dynOPright)

	plug(entFCL.signal(str(arm_right_part)+p2_signal), taskDAMP.task.signal('p1_'+str(arm_right_part)+p2_signal))
	plug(entFCL.signal(p2_signal+str(arm_right_part)), taskDAMP.task.signal('p2_'+str(arm_right_part)+p2_signal))
	plug(dynOPright.signal('jacobian'), taskDAMP.task.signal('jVel_'+str(arm_right_part)+p2_signal))

p2_signal = 'head_1_joint'
counter = 0
for arm_right_part in arm_right_group:

	dynOPright = DynamicOppointModifier(arm_right_part+p2_signal)
	plug(robot.dynamic.signal(arm_right_part), dynOPright.positionIN)
	plug(robot.dynamic.signal('J'+str(arm_right_part)), dynOPright.jacobianIN)
	plug(entFCL.signal('oppoint_'+str(arm_right_part)+p2_signal), dynOPright.transformationSig)
	dynOP_right_collection.append(dynOPright)

	plug(entFCL.signal(str(arm_right_part)+p2_signal), taskDAMP.task.signal('p1_'+str(arm_right_part)+p2_signal))
	plug(entFCL.signal(p2_signal+str(arm_right_part)), taskDAMP.task.signal('p2_'+str(arm_right_part)+p2_signal))
	plug(dynOPright.signal('jacobian'), taskDAMP.task.signal('jVel_'+str(arm_right_part)+p2_signal))

#plug left arm
p2_signal = 'torso_1_joint'
for arm_left_part in arm_left_group:

	dynOPleft = DynamicOppointModifier(arm_left_part+p2_signal)
	plug(robot.dynamic.signal(arm_left_part), dynOPleft.positionIN)
	plug(robot.dynamic.signal('J'+str(arm_left_part)), dynOPleft.jacobianIN)
	plug(entFCL.signal('oppoint_'+str(arm_left_part)+p2_signal), dynOPleft.transformationSig)

	plug(entFCL.signal(str(arm_left_part)+p2_signal), taskDAMP.task.signal('p1_'+str(arm_left_part)+p2_signal))
	plug(entFCL.signal(p2_signal+str(arm_left_part)), taskDAMP.task.signal('p2_'+str(arm_left_part)+p2_signal))
	plug(dynOPleft.signal('jacobian'), taskDAMP.task.signal('jVel_'+str(arm_left_part)+p2_signal))

p2_signal = 'torso_2_joint'
for arm_left_part in arm_left_group:

	dynOPleft = DynamicOppointModifier(arm_left_part+p2_signal)
	plug(robot.dynamic.signal(arm_left_part), dynOPleft.positionIN)
	plug(robot.dynamic.signal('J'+str(arm_left_part)), dynOPleft.jacobianIN)
	plug(entFCL.signal('oppoint_'+str(arm_left_part)+p2_signal), dynOPleft.transformationSig)

	plug(entFCL.signal(str(arm_left_part)+p2_signal), taskDAMP.task.signal('p1_'+str(arm_left_part)+p2_signal))
	plug(entFCL.signal(p2_signal+str(arm_left_part)), taskDAMP.task.signal('p2_'+str(arm_left_part)+p2_signal))
	plug(dynOPleft.signal('jacobian'), taskDAMP.task.signal('jVel_'+str(arm_left_part)+p2_signal))

p2_signal = 'head_1_joint'
for arm_left_part in arm_left_group:

	dynOPleft = DynamicOppointModifier(arm_left_part+p2_signal)
	plug(robot.dynamic.signal(arm_left_part), dynOPleft.positionIN)
	plug(robot.dynamic.signal('J'+str(arm_left_part)), dynOPleft.jacobianIN)
	plug(entFCL.signal('oppoint_'+str(arm_left_part)+p2_signal), dynOPleft.transformationSig)

	plug(entFCL.signal(str(arm_left_part)+p2_signal), taskDAMP.task.signal('p1_'+str(arm_left_part)+p2_signal))
	plug(entFCL.signal(p2_signal+str(arm_left_part)), taskDAMP.task.signal('p2_'+str(arm_left_part)+p2_signal))
	plug(dynOPleft.signal('jacobian'), taskDAMP.task.signal('jVel_'+str(arm_left_part)+p2_signal))

plug(robot.dynamic.signal('torso_1_joint'), taskDAMP.task.debug_in)

# Create basic tasks
taskRW = createEqualityTask('rightWrist', 'arm_right_tool_joint')
taskLW = createEqualityTask('leftWrist', 'arm_left_tool_joint')
taskBASE = createEqualityTask('baseContact','base_joint',10)
taskJL = createJointLimitsTask(500)
safePos = safePosition()
diag = (0,0,0,0,0,0,1.36499,1.49997,0.677897,0.636507,0.170576,0.234007,0.120986,0.0156722,0.0213592,0.687228,0.63189,0.172151,0.260947,0.120986,0.0156722,0.0213592,0.511255,0.520094)
taskWEIGHTS = createWeightsTask(diag)
taskGAZE = createGazeTask('camera_joint')
taskGAZE.goto3D((100,0.0,1.48),10)

basicStack()
gotoNd(taskRW,(0.2,-0.4,1.1),'111111',5)
gotoNd(taskLW,(0.2,0.4,1.1),'111111',5)

push(taskDAMP)
