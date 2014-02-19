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

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("left_hand_ref_pose_filt",PoseStamped, callback_left_hand)
    rospy.Subscriber("head_ref_point_filt",PointStamped, callback_head)

def move_target_left(xyz,quat):
    goal = goalDef(xyz,quat)
    gotoNd(taskLW,goal,'111111',10)

if __name__ == '__main__':
    
    taskLW = MetaTaskKine6d('lw',robot.dynamic,'arm_left_tool_joint','arm_left_tool_joint')
    taskLW.feature.frame('desired')
    
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
    
    taskRL = MetaTaskKine6d('contact_rl',robot.dynamic,'leg_right_6_joint','leg_right_6_joint')
    taskRL.feature.frame('desired')
    taskRL.gain.setConstant(1)
    
    taskLL = MetaTaskKine6d('contact_ll',robot.dynamic,'leg_left_6_joint','leg_left_6_joint')
    taskLL.feature.frame('desired')
    taskLL.gain.setConstant(1)
    
    weights_diag_flag = 0
    if (weights_diag_flag):
        diag = [1,]*robot.dimension
        diag[6] = 10
        diag[7] = 10
        diag[8] = 10
        diag[9] = 10
        diag[10] = 10
        diag[11] = 10
        diag[12] = 10
        diag[13] = 10
        diag[14] = 10
        diag[15] = 10
        diag[16] = 10
        diag[17] = 10
        diag[18] = 10
        diag[19] = 10
        diag = tuple(diag)
    else:
        diag = None
    
    taskWEIGHTS = createWeightsTask(diag,sampleInterval = 10000)
    
    taskCOM = createComIneqTask(dt = 1, referenceInf = (-0.05,-0.05, 0), referenceSup = (0.05,0.05,0))
    
    push(taskJL)
    push(taskCOM)
    solver.addContact(taskRL)
    solver.addContact(taskLL)
    push(taskLW)
    push(taskGAZE)
    push(taskWEIGHTS)
    
    listener()
    
    createRosImport('vector3', robot.dynamic.com, 'sot_controller/com_position')
    ComBordersPublisher(taskCOM.referenceInf.value,taskCOM.referenceSup.value)
    
    rospy.spin()