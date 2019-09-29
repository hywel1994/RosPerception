#!/usr/bin/env python
import rospy

try:
    from usv_gazebo_plugins.msg import Drive
    from usv_gazebo.cfg import usv_gazebo_Config
    is_sim = True
except:
    is_sim = False
    pass

from helper.msg import BaseSensor 
from helper.msg import ObjectArray

from spare_function.cfg import spare_function_Config
from spare_function.msg import spare_function_out
from spare_function.msg import spare_function_para

from dynamic_reconfigure.server import Server

if is_sim:
    from PDcontroller_sim import Controller2Trimaran
else:
    from PDcontroller import Controller2Trimaran

from util import YawLimit

import math
from math import pi

sensor_submsg = [0,0,0,0]
sensor2_submsg = [0,0,0,0]
para_cfg = [0,0,0,0,0,0]
target_submsg = []



def getOutMachPut(msg): #sailboat_message::Mach_msg
    mach_pub = Drive()
    mach_pub.left = msg[0]
    mach_pub.right = msg[1]
    return mach_pub


def getOutput(msg): #spare_function::spare_function_out
    out_pub = spare_function_out()
    out_pub.left = msg[0]
    out_pub.right = msg[1]
    return out_pub


def getOutParaPut(msg):#spare_function::spare_function_para
    para_pubmsg = spare_function_para()
    para_pubmsg.target_yaw = msg[1]
    para_pubmsg.target_x = msg[2]
    para_pubmsg.target_y = msg[3]
    return para_pubmsg


def sensorCallback(msg): #sailboat_message::Sensor_msg
    global sensor_submsg
    sensor_submsg[0] = msg.x
    sensor_submsg[1] = msg.y
    sensor_submsg[2] = msg.yaw
    sensor_submsg[3] = math.sqrt(math.pow(msg.vx, 2)+math.pow(msg.vy, 2))

    sensor2_submsg[0] = msg.x_target
    sensor2_submsg[1] = msg.y_target
    sensor2_submsg[2] = 0.5 * float(sensor2_submsg[2]) + 0.5 * msg.yaw_target#math.atan2(msg.vy_target, msg.vx_target)
    sensor2_submsg[3] = 0.5 * sensor2_submsg[3] + 0.5 * math.sqrt(math.pow(msg.vx_target, 2)+math.pow(msg.vy_target, 2))
    # print ('sensor2_submsg: ', sensor2_submsg)

def trackingCallback_point(msg):
    global target_submsg
    target = []
    for ot in msg.list:
        ot = msg.list[0]
        if ot.semantic_id == 76:
            tmp = [ot.semantic_confidence, ot.world_pose.point.x, ot.world_pose.point.y, ot.velocity, YawLimit(ot.heading)]
            target += [tmp]
        if len(target) > 1:
            target.sort(key=lambda x:x[0])
        if len(target) > 0:
            target_submsg = target[0]
        if len(target_submsg)>0:
            tar_msg = BaseSensor()
            tar_msg.x = target_submsg[1]
            tar_msg.y = target_submsg[2]
            tar_msg.vx = target_submsg[3]
            tar_msg.yaw = target_submsg[4]
            target_pub.publish(tar_msg)
    print ('target_submsg: ', target_submsg)

def trackingCallback_run(msg):
    global target_submsg
    target = []
    for ot in msg.list:
        if ot.semantic_id == 76:
            if ot.velocity < 0:
                ot.heading = -ot.heading
                ot.velocity = -ot.velocity
            tmp = [ot.semantic_confidence, ot.world_pose.point.x, ot.world_pose.point.y, YawLimit(ot.heading), ot.velocity]
            # target += [tmp]
            tar_msg = BaseSensor()
            tar_msg.x = tmp[1]
            tar_msg.y = tmp[2]
            tar_msg.vx = tmp[4]
            tar_msg.yaw = tmp[3]
            target_pub.publish(tar_msg)
    # if len(target) > 1:
    #     target.sort(key=lambda x:x[0])
    # if len(target) > 0:
    #     target_submsg = target[0]
    # if len(target_submsg)>0:
    #     tar_msg = BaseSensor()
    #     tar_msg.x = target_submsg[1]
    #     tar_msg.y = target_submsg[2]
    #     tar_msg.vx = target_submsg[4]
    #     tar_msg.yaw = target_submsg[3]
    #     target_pub.publish(tar_msg)
    print ('target_submsg: ', target_submsg)
    

def getConfigCallback(config, level): #spare_function::spare_function_Config
    global para_cfg
    if (config.set_point == True):
        para_cfg[0] = 1
    else:
        para_cfg[0] = 0
    para_cfg[1] = config.target_yaw
    para_cfg[2] = config.target_x
    para_cfg[3] = config.target_y
    para_cfg[4] = config.max_speed
    para_cfg[5] = config.min_speed
    return config




if __name__ == "__main__":
    rospy.init_node("control", anonymous = True)

    if is_sim:
        mach_pub = rospy.Publisher('self/usv/cmd_drive', Drive, queue_size=5)
    spare_function_pub = rospy.Publisher('spare_function_out', spare_function_out, queue_size=5)
    spare_function_para_pub = rospy.Publisher('spare_function_para', spare_function_para, queue_size=5)
    target_pub = rospy.Publisher('target_pose2', BaseSensor, queue_size=5)

    rospy.Subscriber("/base/sensor", BaseSensor, sensorCallback)
    rospy.Subscriber("/tracking/objects2", ObjectArray, trackingCallback_point)
    config_srv = Server(spare_function_Config, getConfigCallback)

    rate = rospy.Rate(10) 

    controller = Controller2Trimaran()
    target_point = []
    state = 0
    # 0: can't find target
    # 1: find target and go to start area
    # 2: go to target

    try:
        while not rospy.is_shutdown():

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    #finally:
        #close()
    rospy.spin()
