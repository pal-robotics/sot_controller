#!/usr/bin/env python

import roslib
roslib.load_manifest('sot_controller')
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Point
from rospy_tutorials.msg import Floats
import numpy

frame = "/floor_link"

class PublisherCom():
    """
    This class create a node publishing a marker related to the com.
    """
    
    subscriberComBorders = 0
    subscriberComPoint = 0
    publisherComPoint = 0
    com = 0
    comBorders = 0
    ts = 0
    
    def CallbackComPoint(self,data):
        self.com.header.stamp = rospy.Time.now()
        self.com.pose.position.x = data.x
        self.com.pose.position.y = data.y
        self.com.pose.position.z = 0.0
        self.com.pose.orientation.x = 0;
        self.com.pose.orientation.y = 0;
        self.com.pose.orientation.z = 0;
        self.com.pose.orientation.w = 1;
        self.publisherComPoint.publish(self.com)
    
    def CallbackComBorders(self,data):
        vect = numpy.array(data.data,dtype=numpy.float32)
        offs = 3
        x_index = [offs,offs,0,0,offs]
        y_index = [offs+1,1,1,offs+1,offs+1]
        #z_index = [2,offs+2,]*4
        
        x = [vect[i] for i in x_index]
        y = [vect[i] for i in y_index]
        #z = [vect[i] for i in z_index]
        
        points_list = []
        
        for i in range(0,len(x)):
            p = Point()
            p.x = x[i]
            p.y = y[i]
            p.z = 0.0
            points_list.append(p)
        
        self.comBorders.header.stamp = rospy.Time.now()
        self.comBorders.points = points_list
        self.publisherComBorders.publish(self.comBorders)
              
    def __init__(self):
        rospy.init_node('com_publisher')
        
        # Com point
        self.com = Marker()
        self.com.header.frame_id = frame
        self.com.id = 0;
        self.com.action = Marker.ADD
        self.com.type = Marker.SPHERE
        self.com.scale.x = 0.1
        self.com.scale.y = 0.1
        self.com.scale.z = 0.1
        self.com.color.a = 0.8
        self.com.color.r = 0.0
        self.com.color.g = 0.0
        self.com.color.b = 1.0
        
        # Com borders
        self.comBorders = Marker()
        self.comBorders.header.frame_id = frame
        self.comBorders.id = 0;
        self.comBorders.action = Marker.ADD
        self.comBorders.type = Marker.LINE_STRIP
        self.comBorders.scale.x = 0.01
        self.comBorders.color.a = 0.5
        self.comBorders.color.r = 1.0
        self.comBorders.color.g = 0.0
        self.comBorders.color.b = 0.0
        
        self.publisherComPoint = rospy.Publisher("com_position_marker", Marker)
        self.publisherComBorders = rospy.Publisher("com_borders_marker", Marker)
        self.subscriberComPoint = rospy.Subscriber("com_position",Vector3, self.CallbackComPoint)
        self.subscriberComBorders = rospy.Subscriber("com_borders",Floats, self.CallbackComBorders)
        rospy.spin()

if __name__ == '__main__':
    cp = PublisherCom()
