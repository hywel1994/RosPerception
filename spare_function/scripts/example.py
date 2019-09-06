#!/usr/bin/env python
import rospy

from usv_gazebo_plugins.msg import Drive
from usv_gazebo.cfg import usv_gazebo_Config

from helper.msg import BaseSensor 
from helper.msg import ObjectArray

from spare_function.cfg import spare_function_Config
from spare_function.msg import spare_function_out
from spare_function.msg import spare_function_para

from dynamic_reconfigure.server import Server

import math
from math import pi

sensor_submsg = [0,0,0]
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

def trackingCallback(msg):
    global target_submsg
    target = []
    for ot in msg.list:
        if ot.semantic_id == 34 and ot.semantic_confidence > 0.3:
            tmp = [ot.semantic_confidence, ot.world_pose.point.x, ot.world_pose.point.y, ot.velocity, ot.heading]
            target += [tmp]
    if len(target) > 1:
        target.sort(key=lambda x:x[0])
    if len(target) > 0:
        target_submsg = target[0]
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

def calDis(sensor, target):
    disL = math.sqrt(math.pow(sensor[0]-target[0],2)+math.pow(sensor[1]-target[1],2))
    runY = math.atan2(target[1]-sensor[1],target[0]-sensor[0])
    disY = errYawLimit(sensor[2] - runY)
    return disL, disY

# def getTargetYawDis(x_pos, y_pos, target_x, target_y):
    
def errYawLimit(err_yaw):
    '''
    Restrict Yaw Error within [-pi, pi]
    '''
    while err_yaw > pi or err_yaw < -pi:
        if err_yaw > pi:
            err_yaw -= 2*pi
        if err_yaw < -pi:
            err_yaw += 2*pi
    return err_yaw

if __name__ == "__main__":
    rospy.init_node("control", anonymous = True)

    mach_pub = rospy.Publisher('self/usv/cmd_drive', Drive, queue_size=5)
    spare_function_pub = rospy.Publisher('spare_function_out', spare_function_out, queue_size=5)
    spare_function_para_pub = rospy.Publisher('spare_function_para', spare_function_para, queue_size=5)
    
    rospy.Subscriber("/base/sensor", BaseSensor, sensorCallback)
    rospy.Subscriber("/tracking/objects", ObjectArray, trackingCallback)
    config_srv = Server(spare_function_Config, getConfigCallback)

    rate = rospy.Rate(10) 

    start = 0
    current_stage = 0
    target_point = []
    try:
        while not rospy.is_shutdown():
            mach_np = [0,0]
            out_np = [0,0]

            if para_cfg[0] == 1:
                target_yaw = para_cfg[1]
                target_x = para_cfg[2]
                target_y = para_cfg[3]
                #if target_yaw > 1:
                target_point = [[target_x, target_y-10], [target_x, target_y]]
            else:
                # target_point = [[0, -10, 1.57], [0, 0, 1.57]]
                if len(target_submsg) == 0:
                    v = 0
                    rudder = 0.02
                    mach_np[0] = v + rudder
                    mach_np[1] = v - rudder
                    mach_pubmsg = getOutMachPut(mach_np)
                    mach_pub.publish(mach_pubmsg)
                    rate.sleep()
                    continue
                else:
                    target_point = [[target_submsg[1]-2, target_submsg[2]-10], [target_submsg[1]-2, target_submsg[2]]]
            
            print ('target_point', target_point)
            
            if start >= len(target_point):
                print ('find target')
                rate.sleep()
                continue
            current_target = target_point[start]
            disL, disY = calDis(sensor_submsg, current_target)

            print ('sensor_submsg', sensor_submsg, 'disL, disY: ', disL, disY, 'start, current_stage: ', start, current_stage)
            
            
            #if start == len(target_point)-1:
            tmp_target_disL = 0.1
            tmp_target_disY = 0.1
            # else:
            #     tmp_target_disL = 0.3
            #     tmp_target_disY = 0.3

            if current_stage == 0:
                if abs(disY) > tmp_target_disY:
                    v = 0
                    rudder = 0.05*(disY)
                    mach_np[0] = v + rudder
                    mach_np[1] = v - rudder
                else:
                    current_stage += 1
            
            if current_stage == 1:
                if disL > tmp_target_disL:
                    if abs(disY) < tmp_target_disY*5 or disL < tmp_target_disL*5:
                        if start == len(target_point)-1:
                            v = min(0.02*disL, para_cfg[4])
                        else:
                            v = min(0.05*disL, para_cfg[4])
                        rudder = 0.01*(disY)
                        mach_np[0] = v + rudder
                        mach_np[1] = v - rudder
                    else:
                        current_stage -= 1
                else:
                    current_stage = 0
                    start += 1
            
            
            print ('mach_np: ', mach_np)
            mach_pubmsg = getOutMachPut(mach_np)
            out_pubmsg = getOutput(out_np)
            para_pubmsg = getOutParaPut(para_cfg)

            mach_pub.publish(mach_pubmsg)
            spare_function_pub.publish(out_pubmsg)
            spare_function_para_pub.publish(para_pubmsg)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    #finally:
        #close()
    rospy.spin()
