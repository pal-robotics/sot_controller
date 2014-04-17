'''
In this demo the joint weights task is used.
The task objective is to align the camera_link axis with a point placed on the right side of the robot.
By using the joint weights task we enforce the solver to find a solution accordingly to the weights assigned to each joint e.g.
the torso will be moved less then the head.
Note: the weights are applied on the joints velocities not on the position.

The joint weights task formulation is the following:
W * qdot(k) = K * [qdot(k-1) + (q_des - q)/dt]
with:
    - W weights matrix, can be specified as a constant matrix or using the robot inertia
    - qdot(k) joints velocity (to compute in the solver)
    - qdot(k-1) current joints velocity
    - K gain factor
    - q_des desired joints configuration
    - q current joints configuration
    - dt sample time
This task can be used in two different ways:
    - to have a weighted metric in the IK computation
      W * qdot(k) = K * qdot(k-1) with 0 < K <= 1 (K is used to keep a low velocity)
    - to define a task in the joints space
      W * qdot(k) = K * [qdot(k-1) + (q_des - q)/dt]

The stack is composed by:
    - joints limits constraint
    - base link contact (this constraint fixes the robot free flyer)
    - gaze task 
    - task weights

@author: Gennaro Raiola, Karsten Knese
'''
from sot_ros_api import *

xyz = numpy.array([0,-100,1.483])

taskBASE = createEqualityTask('baseContact','base_joint',10)
taskGAZE = createGazeTask('camera_joint')
taskJL = createJointLimitsTask(10)

weights_diag_flag = 1
if (weights_diag_flag):
    diag = (0,0,0,0,0,0,1.36499,1.49997,0.677897,0.636507,0.170576,0.234007,0.120986,0.0156722,0.0213592,0.687228,0.63189,0.172151,0.260947,0.120986,0.0156722,0.0213592,0.511255,0.520094)
else:
    diag = None

taskWEIGHTS = createWeightsTask(diag,gain=0.001)

push(taskJL)
solver.addContact(taskBASE)
push(taskGAZE)
push(taskWEIGHTS)

taskGAZE.goto3D((xyz[0],xyz[1],xyz[2]),10)
