#!/usr/bin/env python
# for ros
import rospy
from helper.msg import BaseSensor 
from helper.msg import ObjectArray
from helper.msg import Object
from geometry_msgs.msg import PointStamped

count = 0
def sensorCallback(msg):
    global count
    d_object = Object()
    
    d_object.semantic_name = 'boat'
    d_object.semantic_id = 76
    d_object.semantic_confidence = 0.7
    d_object.world_pose.header.stamp = rospy.Time.now()
    d_object.world_pose.point.x = msg.x_target
    d_object.world_pose.point.y = msg.y_target
    d_object.world_pose.point.z = 0
    d_object.semantic_confidence = 0.6
    d_object.width = 1
    d_object.length = 1
    d_object.height = 1
    d_object.orientation = 1
    d_object.is_new_track = True
    d_object.r = 0
    d_object.g = 0
    d_object.b = 0
    d_object.a = 0.75
    d_object_list = ObjectArray()
    d_object_list.header.stamp = rospy.Time.now()
    d_object_list.list.append(d_object)
    if count == 0:
        print ('detection x:', msg.x_target, ' y: ' ,msg.y_target)
        object_array_pub.publish(d_object_list)
    
    count += 1
    if count == 10:
        count = 0

if __name__ == "__main__":
    rospy.init_node("fake_detection", anonymous = True)
    object_array_pub = rospy.Publisher('detection/objects', ObjectArray, queue_size=5)
    rospy.Subscriber("/base/sensor", BaseSensor, sensorCallback)
  
    rospy.spin()
