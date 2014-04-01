#!/usr/bin/env python
from sot_ros_api import *

from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PoseStamped, PointStamped
from dynamic_graph.sot.core.meta_task_joint_weights import MetaTaskJointWeights

def ComBordersPublisher(refInf,refSup):
    pub = rospy.Publisher('sot_controller/com_borders', numpy_msg(Floats))
    a = numpy.concatenate((numpy.array(refInf,dtype=numpy.float32),numpy.array(refSup,dtype=numpy.float32)))
    #r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(a)
    #r.sleep()

def callback_head(data):
    xyz = numpy.array([data.point.x , data.point.y , data.point.z])
    taskGAZE.goto3D((xyz[0],xyz[1],xyz[2]))

def callback_left_hand(data):
    quat = numpy.array([data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z , data.pose.orientation.w])
    xyz = numpy.array([data.pose.position.x , data.pose.position.y , data.pose.position.z])
    move_target_left(xyz,quat)

def callback_right_hand(data):
    quat = numpy.array([data.pose.orientation.x , data.pose.orientation.y , data.pose.orientation.z , data.pose.orientation.w])
    xyz = numpy.array([data.pose.position.x , data.pose.position.y , data.pose.position.z])
    move_target_right(xyz,quat)

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("left_hand_ref_pose_filt",PoseStamped, callback_left_hand)
    rospy.Subscriber("right_hand_ref_pose_filt",PoseStamped, callback_right_hand)
    rospy.Subscriber("head_ref_point_filt",PointStamped, callback_head)

def move_target_left(xyz,quat):
    goal = goalDef(xyz,quat)
    gotoNd(taskLW,goal,'111111',10)

def move_target_right(xyz,quat):
    goal = goalDef(xyz,quat)
    gotoNd(taskRW,goal,'111111',10)

if __name__ == '__main__':
    
    taskLW = MetaTaskKine6d('lw',robot.dynamic,'arm_left_tool_joint','arm_left_tool_joint')
    taskLW.feature.frame('current')
    
    taskRW = MetaTaskKine6d('rw',robot.dynamic,'arm_right_tool_joint','arm_right_tool_joint')
    taskRW.feature.frame('current')
    
    taskGAZE = createGazeTask("camera_joint")
    
    robot.dynamic.upperJl.recompute(0)
    robot.dynamic.lowerJl.recompute(0)
    taskJL = TaskJointLimits('taskJL')
    plug(robot.dynamic.position,taskJL.position)
    taskJL.controlGain.value = 1
    taskJL.referenceInf.value = robot.dynamic.lowerJl.value
    taskJL.referenceSup.value = robot.dynamic.upperJl.value
    taskJL.dt.value = 1
    taskJL.selec.value = toFlags(range(6,robot.dimension))
    
    taskRL = MetaTaskKine6d('contact_rl',robot.dynamic,'right_sole_joint','right_sole_joint')
    taskRL.feature.frame('current')
    taskRL.gain.setConstant(1)
    
    taskLL = MetaTaskKine6d('contact_ll',robot.dynamic,'left_sole_joint','left_sole_joint')
    taskLL.feature.frame('current')
    taskLL.gain.setConstant(1)
    
    weights_diag_flag = 1
    if (weights_diag_flag):
        diag = numpy.array([ 8.00916  ,  8.00916  ,  8.00916  ,  2.96527  ,  2.85807  ,
        1.54464  ,  0.469273 ,  1.20164  ,  1.12471  ,  0.517821 ,
        0.109894 ,  0.0615758,  0.469273 ,  1.19488  ,  1.11749  ,
        0.509942 ,  0.109894 ,  0.0615758,  1.31341  ,  1.50104  ,
        0.683272 ,  0.639302 ,  0.178914 ,  0.236697 ,  0.120986 ,
        0.0156722,  0.0213592,  0.692531 ,  0.634706 ,  0.180416 ,
        0.263362 ,  0.120986 ,  0.0156722,  0.0213592,  0.511262 ,
        0.520098 ])
        diag = tuple(diag)
    else:
        diag = None
    
    taskWEIGHTS = createWeightsTask(diag, dt = 1, gain = 1)
    
    offset = 0.075
    taskCOM = createComIneqTask(dt = 1, referenceInf = (-0.3,-0.3+offset, 0), referenceSup = (0.3,0.3+offset,0))
    
    push(taskJL)
    push(taskCOM)
    solver.addContact(taskLL)
    push(taskLW)
    push(taskGAZE)
    
    listener()
    
    createRosImport('vector3', robot.dynamic.com, 'sot_controller/com_position')
    ComBordersPublisher(taskCOM.referenceInf.value,taskCOM.referenceSup.value)
    
    rospy.spin()