#!/usr/bin/env python
# for ros
import rospy
from helper.msg import BaseSensor 
from helper.msg import ObjectArray
from helper.msg import Object

import math

def sensorCallback(msg):
    d_object = Object()
    d_object.semantic_name = 'boat'
    d_object.semantic_id = 76
    d_object.world_pose.point.x = msg.x_target
    d_object.world_pose.point.y = msg.y_target
    d_object.world_pose.point.z = 0
    d_object.velocity = math.sqrt(pow(msg.vx_target, 2)+pow(msg.vy_target, 2))
    d_object.heading = msg.yaw_target
    d_object.semantic_confidence = 0.6
    d_object.width = 1
    d_object.length = 1
    d_object.height = 1
    d_object.is_new_track = True
    d_object_list = ObjectArray()
    d_object_list.list.append(d_object)

    object_array_pub.publish(d_object_list)

if __name__ == "__main__":
    rospy.init_node("fake_tracking", anonymous = True)
    object_array_pub = rospy.Publisher('/tracking/objects', ObjectArray, queue_size=5)
    rospy.Subscriber("/base/sensor", BaseSensor, sensorCallback)
  
    rospy.spin()
