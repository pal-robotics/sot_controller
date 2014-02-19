#!/usr/bin/env python
from sot_ros_api import *

from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

def ComBordersPublisher(refInf,refSup):
    pub = rospy.Publisher('sot_controller/com_borders', numpy_msg(Floats))
    a = numpy.concatenate((numpy.array(refInf,dtype=numpy.float32),numpy.array(refSup,dtype=numpy.float32)))
    #r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(a)
    #r.sleep()

if __name__ == '__main__':
    rospy.init_node('reemc_equilibrium')
    
    taskRW = createEqualityTask('rightWrist', 'arm_right_tool_joint', gain = 10)
    taskLW = createEqualityTask('leftWrist', 'arm_left_tool_joint', gain = 10)
    
    taskGAZE = createGazeTask("head_2_joint")
    taskGAZE.gain.setConstant(1)
    
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
    
    taskWEIGHTS = createWeightsTask(diag,gain = 100)
    
    offset = 0.075
    taskCOM = createComIneqTask(dt = 1, referenceInf = (-0.1,-0.1+offset, 0), referenceSup = (0.1,0.1+offset,0))
    
    push(taskJL)
    push(taskCOM)
    solver.addContact(taskLL)
    push(taskLW)
    push(taskRW)
    push(taskGAZE)
    push(taskWEIGHTS)
    
    createRosImport('vector3',robot.dynamic.com, 'sot_controller/com_position')
    createRosExport('matrixHomoStamped',taskLW.featureDes.position,'left_hand_ref_pose_filt')
    createRosExport('matrixHomoStamped',taskRW.featureDes.position,'right_hand_ref_pose_filt')
    createRosExport('vector3',taskGAZE.proj.point3D,'head_ref_point_filt')
        
    ComBordersPublisher(taskCOM.referenceInf.value,taskCOM.referenceSup.value)
    
    rospy.spin()