#!/usr/bin/python
from sot_ros_api import *
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

def ComBordersPublisher(refInf,refSup):
    pub = rospy.Publisher('sot_controller/com_borders', numpy_msg(Floats))
    a = numpy.concatenate((numpy.array(refInf,dtype=numpy.float32),numpy.array(refSup,dtype=numpy.float32)))
    #r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(a)
    #r.sleep()

if __name__ == '__main__':
    
    rospy.init_node('com_borders_publisher')
    
    taskLW = MetaTaskKine6d('lw',robot.dynamic,'arm_left_tool_joint','arm_left_tool_joint')
    taskLW.feature.frame('desired')
    
    taskGAZE = createGazeTask("camera_joint")
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
    
    taskRL = MetaTaskKine6d('contact_rl',robot.dynamic,'leg_right_6_joint','leg_right_6_joint')
    taskRL.feature.frame('desired')
    taskRL.gain.setConstant(1)
    
    taskLL = MetaTaskKine6d('contact_ll',robot.dynamic,'leg_left_6_joint','leg_left_6_joint')
    taskLL.feature.frame('desired')
    taskLL.gain.setConstant(1)
    
    weights_diag_flag = 1
    if (weights_diag_flag):
        diag = [1,]*robot.dimension
        diag[6] = 10
        diag[7] = 10
        diag[8] = 10
        diag[12] = 10
        diag[13] = 10
        diag[14] = 10
        diag[18] = 10
        diag[19] = 10
        diag = tuple(diag)
    else:
        diag = None
    
    taskWEIGHTS = MetaTaskJointWeights('weights',robot,diag)
    
    taskCOM = createComIneqTask(dt = 1, referenceInf = (-0.1,-0.1, 0), referenceSup = (0.1,0.1,0))
    
    push(taskJL)
    push(taskCOM)
    solver.addContact(taskRL)
    solver.addContact(taskLL)
    push(taskLW)
    push(taskGAZE)
    
    gotoNd(taskLW,((0.2,0.4,0.5)),'111',1)
    taskGAZE.goto3D(((0.2,0.4,0.5)),1)
    
    createRosImport('vector3', robot.dynamic.com, 'sot_controller/com_position')
    ComBordersPublisher(taskCOM.referenceInf.value,taskCOM.referenceSup.value)