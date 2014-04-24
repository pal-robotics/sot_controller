'''
@author: Karsten Knese, Gennaro Raiola
'''
from sot_ros_api import *

from dynamic_graph.dynamic_graph_fcl import DynamicGraphFCL
from dynamic_graph.sot.core.meta_task_dynamic_velocity_damping import MetaTaskDynamicVelocityDamping
from dynamic_graph.sot.core.dyn_oppoint_modifier import DynamicOppointModifier

def basicStack():
	# Basic stack
	push(taskJL)
	solver.addContact(taskBASE)

collision_objects = ['arm_right_7_joint','torso_2_joint', 'torso_1_joint', 'arm_left_7_joint', 'arm_right_5_joint', 'arm_left_5_joint', 'arm_right_3_joint', 'arm_left_3_joint', 'head_1_joint', 'base_joint']
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

# just for triggering computation
createRosImport('matrixHomo', entFCL.head_1_jointarm_right_7_joint, 'rviz_marker_closest_points/avoid_head_1_jointarm_right_7_joint')
createRosImport('matrixHomo', entFCL.torso_1_jointarm_right_7_joint, 'rviz_marker_closest_points/avoid_torso_1_jointarm_right_7_joint')
createRosImport('matrixHomo', entFCL.torso_2_jointarm_right_7_joint, 'rviz_marker_closest_points/avoid_torso_2_jointarm_right_7_joint')
createRosImport('matrixHomo', entFCL.base_jointarm_right_7_joint, 'rviz_marker_closest_points/avoid_base_jointarm_right_7_joint')
createRosImport('matrixHomo', entFCL.arm_right_3_jointarm_right_7_joint, 'rviz_marker_closest_points/avoid_arm_right_3_jointarm_right_7_joint')
createRosImport('matrixHomo', entFCL.arm_right_5_jointarm_right_7_joint, 'rviz_marker_closest_points/avoid_arm_right_5_jointarm_right_7_joint')
createRosImport('matrixHomo', entFCL.arm_right_7_jointarm_left_7_joint, 'rviz_marker_closest_points/avoid_arm_right_7_jointarm_left_7_joint')
createRosImport('matrixHomo', entFCL.arm_left_3_jointarm_right_7_joint, 'rviz_marker_closest_points/avoid_arm_left_3_jointarm_right_7_joint')
createRosImport('matrixHomo', entFCL.arm_left_5_jointarm_right_7_joint, 'rviz_marker_closest_points/avoid_arm_left_5_jointarm_right_7_joint')
createRosImport('matrixHomo', entFCL.arm_left_7_jointarm_right_7_joint, 'rviz_marker_closest_points/avoid_arm_left_7_jointarm_right_7_joint')

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

push(taskRW)
push(taskLW)
