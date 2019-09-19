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

import math
from math import pi

sensor_submsg = [0,0,0,0,0]
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
    sensor_submsg[3] = msg.vx
    sensor_submsg[4] = msg.vy

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


def calPoint(target):
    dis = 4
    dis_2 = 10

    dis_x = math.cos(target[2]*pi/180)*dis
    dis_y = math.sin(target[2]*pi/180)*dis

    run_yaw = (target[2]-90)*pi/180

    dis_x_2 = math.cos(run_yaw)*dis_2
    dis_y_2 = math.sin(run_yaw)*dis_2

    pos = [target[0]+dis_x, target[1]+dis_y]
    pos_farther = [pos[0]+dis_x_2, pos[1]+dis_y_2]
    pos_close = [pos[0]-dis_x_2+dis_x/2, pos[1]-dis_y_2+dis_y/2]

    return pos_farther, pos, pos_close, run_yaw

def calPosDis(pos_close, self_pos):
    disL = math.sqrt(math.pow(pos_close[0]-self_pos[0],2)+math.pow(pos_close[1]-self_pos[1],2))
    target_yaw = math.atan2(pos_close[1]-self_pos[1],pos_close[0]-self_pos[0])

    target_u = disL*0.5
    return disL, target_yaw, target_u

def calLosDis(target, point, pos_farther, self_pos, run_yaw):

    dis_L_y = ((self_pos[0]-point[0])*(point[1]-pos_farther[1])-(self_pos[1]-point[1])*(point[0]-pos_farther[0])) / math.sqrt(math.pow(point[1]-pos_farther[1], 2)+math.pow(point[0]-pos_farther[0], 2))
    dis_L_u = - ((self_pos[0]-point[0])*(point[1]-target[1])-(self_pos[1]-point[1])*(point[0]-target[0])) / math.sqrt(math.pow(point[1]-target[1], 2)+math.pow(point[0]-target[0], 2))
    
    target_yaw = run_yaw - math.atan2(dis_L_y, max(5, dis_L_u))
    target_u = dis_L_u*0.5

    return dis_L_y, dis_L_u, target_yaw, target_u



if __name__ == "__main__":
    rospy.init_node("control", anonymous = True)
    if is_sim:
        mach_pub = rospy.Publisher('self/usv/cmd_drive', Drive, queue_size=5)
    spare_function_pub = rospy.Publisher('spare_function_out', spare_function_out, queue_size=5)
    spare_function_para_pub = rospy.Publisher('spare_function_para', spare_function_para, queue_size=5)
    
    rospy.Subscriber("/base/sensor", BaseSensor, sensorCallback)
    rospy.Subscriber("/tracking/objects", ObjectArray, trackingCallback)
    config_srv = Server(spare_function_Config, getConfigCallback)

    rate = rospy.Rate(10) 

    controller = Controller2Trimaran()

    state = 0
    # 0: can't find target
    # 1: find target and go to start area
    # 2: go to target
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
                target_point = [target_x, target_y]
            else:
                # target_point = [15, -5, 180]
                if len(target_submsg) > 0:
                    target_point = [target_submsg[1], target_submsg[2], 180]
                    pos_farther, pos, pos_close, run_yaw = calPoint(target_point)


            if state == 0:
                left_motor, right_motor = 0.02, -0.02
                if len(target_point) > 0:
                    state += 1
            
            if state == 1:

                disL, target_yaw, target_u = calPosDis(pos_close, sensor_submsg)
                
                self_u = math.sqrt(math.pow(sensor_submsg[3], 2)+math.pow(sensor_submsg[4], 2))
                left_motor,right_motor = controller.outputSignal(target_yaw, sensor_submsg[2], target_u, self_u)

                if (disL < 1):
                    state += 1
            
            if state == 2:
                print ('pos_farther, pos, pos_close', pos_farther, pos, pos_close)
                dis_L_y, dis_L_u, target_yaw, target_u = calLosDis(target_point, pos, pos_farther, sensor_submsg, run_yaw)

                if (dis_L_u < 1):
                    print ('find target')
                    rate.sleep()
                    continue
                
                print ('run_yaw: ', run_yaw)
                print ('dis_L_y, dis_L_u: ', dis_L_y, dis_L_u)

                print ('target_yaw, target_u: ', target_yaw, target_u)
                
                self_u = math.sqrt(math.pow(sensor_submsg[3], 2)+math.pow(sensor_submsg[4], 2))
                left_motor,right_motor = controller.outputSignal(target_yaw, sensor_submsg[2], target_u, self_u)

            mach_np = [left_motor,right_motor]
            
            print ('mach_np: ', mach_np)
            if is_sim:
                mach_pubmsg = getOutMachPut(mach_np)
            out_pubmsg = getOutput(out_np)
            para_pubmsg = getOutParaPut(para_cfg)
            
            if is_sim:
                mach_pub.publish(mach_pubmsg)
            spare_function_pub.publish(out_pubmsg)
            spare_function_para_pub.publish(para_pubmsg)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    #finally:
        #close()
    rospy.spin()
