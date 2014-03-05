'''
@author: Karsten Knese, Gennaro Raiola
Import notes to consider:
because of some instability of the solver in terms of numerical errors
it's highly recommended by our side to stick to the following gains:

- joints limits have to be not less than 500
- a full positioning in terms of Kinematic Meta tasks have to be provided: '111 111'
- the task gain can be variable but a working solution is a simple gain of 1

This is a basic demo with reem or reemc robot using the stack of tasks.
The stack is composed by:
    - joints limits constraint
    - base link contact (this constraint fixes the robot free flyer)
    - reach-task with right wrist
'''
from sot_ros_api import *

def speedUp(gainvalue):
    taskRW.task.controlGain.value = gainvalue
    taskLW.task.controlGain.value = gainvalue

def basicStack():
    # Basic stack
    push(taskJL)
    solver.addContact(taskBASE)
    
# Follow two trajectories with right/left wrist and the gaze
def followBalls():
    solver.clear()

    createRosExport('matrixHomoStamped',taskRW.featureDes.position,'/sot_filters/right_wrist_pose')
    createRosExport('matrixHomoStamped',taskLW.featureDes.position,'/sot_filters/left_wrist_pose')
    taskRW.task.controlGain.value = 0.1
    taskLW.task.controlGain.value = 0.1
    
    # This line is necessary for the initializations of the signals related to the gaze
    # This line is necessary for the initializations of the signals related to the gaze
#     createRosExport('vector3Stamped',taskGAZE.proj.point3D,'/sot_filters/gaze_position')
    time.sleep(5)
    taskGAZE.goto3D((100,0.0,1.48),1)
    basicStack()
    push(taskRW)
    push(taskLW)
#     push(taskGAZE)
    push(taskWEIGHTS)

# Follow one trajectory and avoid a ball tracked using the reem cameras
def avoidBall():
    solver.clear()

    basicStack()
    taskRW.task.controlGain.value = 0.1
    taskLW.task.controlGain.value = 0.1
    createRosExport('matrixHomoStamped',taskRW.featureDes.position,'/sot_filters/right_wrist_pose')
    createRosExport('vector3Stamped',taskGAZE.proj.point3D,'/sot_filters/ball_position')
    #taskRW.task.controlGain.value = 10
    taskGAZE.goto3D((100,0.0,1.48),1)
    time.sleep(5)

    push(taskGAZE)
    createRosExport('matrixHomoStamped',taskDAMP.task.p2,'/sot_filters/ball_pose')
    push(taskDAMP)
    push(taskRW)
    push(taskWEIGHTS)
    createRosImport('matrixHomoStamped', taskDAMP.task.p1, 'rviz_marker/position1')
    createRosImport('matrixHomoStamped', taskDAMP.task.p2, 'rviz_marker/position2')
    createRosImport('vector3Stamped', taskDAMP.task.n, 'rviz_marker/unitvec')
    createRosImport('double', taskDAMP.task.ds, 'rviz_marker/ds')
    createRosImport('double', taskDAMP.task.di, 'rviz_marker/di')


# Create basic tasks
taskRW = createEqualityTask('rightWrist', 'arm_right_tool_joint')
taskLW = createEqualityTask('leftWrist', 'arm_left_tool_joint')
taskBASE = createEqualityTask('baseContact','base_joint',10)
taskJL = createJointLimitsTask(500)
taskGAZE = createGazeTask('camera_joint')
safePos = safePosition()
diag = (0,0,0,0,0,0,1.36499,1.49997,0.677897,0.636507,0.170576,0.234007,0.120986,0.0156722,0.0213592,0.687228,0.63189,0.172151,0.260947,0.120986,0.0156722,0.0213592,0.511255,0.520094)
taskWEIGHTS = createWeightsTask(diag)
# Create the inequalities
taskDAMP = createVelocityDampingTask('velDamp', 'arm_right_tool_joint', None, 0.2, 0.15)
taskDAMP.task.controlGain.value = 1

