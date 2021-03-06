#!/usr/bin/env python  

#from joint_states_listener.srv import *
from sensor_msgs.msg import JointState

import rospy
import tf
import numpy
import math

def rotx(angle):
    m = numpy.matrix([[1,0,0],[0,numpy.cos(angle),-numpy.sin(angle)],[0,numpy.sin(angle),numpy.cos(angle)]])
    return m

def roty(angle):
    m = numpy.matrix([[numpy.cos(angle),0,numpy.sin(angle)],[0,1,0],[-numpy.sin(angle),0,numpy.cos(angle)]])
    return m

def rotz(angle):
    m = numpy.matrix([[numpy.cos(angle),-numpy.sin(angle),0],[numpy.sin(angle),numpy.cos(angle),0],[0,0,1]])
    return m

def mat2rpy(m):
    rpy = numpy.zeros(3)
    rpy[0] = math.atan2(m[1,0],m[0,0])
    rpy[1] = math.atan2(-m[2,0],math.sqrt(m[2,1]**2+m[2,2]**2))
    rpy[2] = math.atan2(m[2,1],m[2,2])
    return rpy


def mat2quat(m):
    q = numpy.zeros(4)
    q[3] = 0.5 * math.sqrt(m[0,0]+m[1,1]+m[2,2]+1)
    q[0] = 0.5 * numpy.sign(m[2,1]-m[1,2])*math.sqrt(m[0,0]-m[1,1]-m[2,2]+1)
    q[1] = 0.5 * numpy.sign(m[0,2]-m[2,0])*math.sqrt(m[1,1]-m[2,2]-m[0,0]+1)
    q[2] = 0.5 * numpy.sign(m[1,0]-m[0,1])*math.sqrt(m[2,2]-m[0,0]-m[1,1]+1)
    return q

def invMat(mat):
    return numpy.linalg.inv(mat)

def homTransf(xyz = [0,0,0],quat=[0,0,0,1]):
    goal = numpy.matrix([[0 , 0, 0, xyz[0]], [0,  0, 0, xyz[1]], [0, 0, 0, xyz[2]], [0, 0, 0, 1]])
    goal_r = quat2mat(quat)
    goal[0:3,0:3] = goal_r
    return goal

def rpy2quat(rpy):
    m = rpy2mat(rpy)
    q = mat2quat(m)
    return q

def quat2mat(q):
    w = q[3]
    x = q[0]
    y = q[1]
    z = q[2]
    m = numpy.matrix( [[ 2*(w**2+x**2)-1 , 2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),2*(w**2+y**2)-1,2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),2*(w**2+z**2)-1]])
    return m

def rpy2mat(rpy):
    m = rotz(rpy[0])*roty(rpy[1])*rotx(rpy[2])
    return m


class FloorFramePublisher():    
    
    frame_parent = ""
    frame_child = ""
    br = []
    
    """
	Define a frame attached to the floor in the base frame. 
	To define this frame the inverse of free-flyer pose is used.
	In this way we have defined a "fixed" frame useful for the visualization in rviz.
    """
    
    #callback function: when a joint_states message arrives, save the values
    def ffpose_callback(self, msg):
        xyz = msg.position[0:3]
        rpy = msg.position[3:6]
        quat = rpy2quat(rpy)
        ffposeHomTransf = homTransf(xyz,quat)
        floorHomTransf = invMat(ffposeHomTransf)
        xyz = floorHomTransf[0:3,3]
        quat = mat2quat(floorHomTransf)
        self.br.sendTransform(xyz,quat,rospy.Time.now(),self.frame_child,self.frame_parent)
        
    def __init__(self,frame_parent = "base_link", frame_child = "floor_link"):
        self.frame_parent = frame_parent
        self.frame_child = frame_child
        rospy.init_node('floor_frame_publisher')
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber('/dynamic_graph/joint_states', JointState, self.ffpose_callback)
        rospy.spin()

if __name__ == '__main__':
    fp = FloorFramePublisher()
    
    
    
    """
class FloorFramePublisher():

    #callback function: when a joint_states message arrives, save the values
    def ffpose_callback(self, msg):
        xyz = msg.position[0:3]
        quat = msg.position[3:6]
    
    def __init__(self,xyz = (0.0,0.0,0.0),quat = (0.0,0.0,0.0,1),frame_parent = "base_link", frame_child = "floor_link"):
        rospy.init_node('floor_frame_publisher')
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10)
        if rospy.has_param("ffpose"):
            ffpose = rospy.get_param("ffpose")
            xyz = ffpose[0:3]
            rpy = ffpose[3:6]
            quat = rpy2quat(rpy)
            ffposeHomTransf = homTransf(xyz,quat)
            floorHomTransf = invMat(ffposeHomTransf)
            xyz = floorHomTransf[0:3,3]
            quat = mat2quat(floorHomTransf)
            rospy.Subscriber('dynamic_graph/joint_states', JointState, self.ffpose_callback)
        while not rospy.is_shutdown():
            br.sendTransform(xyz,quat,rospy.Time.now(),frame_child,frame_parent)
            rate.sleep()

if __name__ == '__main__':
    fp = FloorFramePublisher()
    """
