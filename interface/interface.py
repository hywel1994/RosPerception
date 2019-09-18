#!/usr/bin/env python
# for ros
import rospy
import tf
from spare_function.msg import spare_function_out
from helper.msg import BaseSensor 

from math import pi, sin, cos, sqrt
import time

from msgdev import MsgDevice


POS_X = 0
POS_Y = 1
YAW = 2
YAW_SPEED = 3
SPD = 4
SPD_DIR = 5

class Interface(object):

    def __init__(self, sub_addr, ahrs_port, gnss_port, motor_port):
        self.dev = MsgDevice()
        self.dev.open()
        self.dev.sub_connect(sub_addr+':'+ahrs_port)
        self.dev.sub_add_url('ahrs.roll')
        self.dev.sub_add_url('ahrs.pitch')
        self.dev.sub_add_url('ahrs.yaw')
        self.dev.sub_add_url('ahrs.roll_speed')
        self.dev.sub_add_url('ahrs.pitch_speed')
        self.dev.sub_add_url('ahrs.yaw_speed')
        self.dev.sub_add_url('ahrs.acce_x')
        self.dev.sub_add_url('ahrs.acce_y')
        self.dev.sub_add_url('ahrs.acce_z')

        self.dev.sub_connect(sub_addr+':'+gnss_port)
        self.dev.sub_add_url('gps.time')
        self.dev.sub_add_url('gps.posx')
        self.dev.sub_add_url('gps.posy')
        self.dev.sub_add_url('gps.posz')
        self.dev.sub_add_url('gps.stdx')
        self.dev.sub_add_url('gps.stdy')
        self.dev.sub_add_url('gps.stdz')
        self.dev.sub_add_url('gps.satn')
        self.dev.sub_add_url('gps.hspeed')
        self.dev.sub_add_url('gps.vspeed')
        self.dev.sub_add_url('gps.track')

        self.dev.pub_bind('tcp://0.0.0.0:'+motor_port)

    def receive(self, *args):
        data = []
        for i in args:
            data.append(self.dev.sub_get1(i))
        return data

    def Motor_send(self, left_motor, right_motor):
        self.dev.pub_set1('pro.left.speed', left_motor)
        self.dev.pub_set1('pro.right.speed', right_motor)


def ship_initialize(USE_TLG001, USE_TLG002):
    if USE_TLG001:
        sub_addr1 = 'tcp://192.168.1.150'  # 'tcp://127.0.0.1'
        ahrs_port1 = '55005'
        gnss_port1 = '55004'
        motor_port1 = '55002'
        interface001 = Interface(sub_addr1, ahrs_port1, gnss_port1, motor_port1)
    else:
        interface001 = None

    if USE_TLG002:
        sub_addr2 = 'tcp://192.168.1.152'  # 'tcp://127.0.0.1'
        ahrs_port2 = '55205'
        gnss_port2 = '55204'
        motor_port2 = '55202'
        interface002 = Interface(sub_addr2, ahrs_port2, gnss_port2, motor_port2)
    else:
        interface002 = None

    return interface001, interface002


class synchronizer:
    def __init__(self, rate=10, USE_TLG001=True, USE_TLG002=False):

        self.sensor_pub = rospy.Publisher('/base/sensor', BaseSensor, queue_size=1)
        self.mach_sub = rospy.Subscriber("/spare_function_out", spare_function_out, self.spareFunctionOutCallback, queue_size=2)

        self.br_boat_world = tf.TransformBroadcaster()
        self.br_lidar_boat = tf.TransformBroadcaster()
        # self.br_lidar_boat2 = tf.TransformBroadcaster()
        self.br_camera_boat = tf.TransformBroadcaster()
        
        rate = rospy.Rate(rate)
        self.USE_TLG001 = USE_TLG001
        self.USE_TLG002 = USE_TLG002
        self.interface001, self.interface002 = ship_initialize(self.USE_TLG001, self.USE_TLG002)

        try:
            while not rospy.is_shutdown():
                print ('process')
                self_state = self.interface001.receive('gps.posx', 'gps.posy', 'ahrs.yaw',
                                                    'ahrs.yaw_speed', 'gps.hspeed',
                                                    'gps.stdx', 'gps.stdy', 'gps.track')
                target_state = self.interface002.receive('gps.posx', 'gps.posy', 'ahrs.yaw',
                                                    'ahrs.yaw_speed', 'gps.hspeed',
                                                    'gps.stdx', 'gps.stdy', 'gps.track')

                self.general_callback(self_state[POS_X], self_state[POS_Y],
                                        self_state[YAW], self_state[SPD],
                                        target_state[POS_X], target_state[POS_Y],
                                        target_state[YAW], target_state[SPD])
                rate.sleep()
        finally:
            self.interface001.Motor_send(0, 0)
            self.interface001.dev.close()
            self.interface002.dev.close()
            print('everything closed')

    def spareFunctionOutCallback(self, msg):
        self.interface001.Motor_send(msg.left, msg.right)
        print ('send motor', msg.left, msg.right)

        
    def general_callback(self, north, east, yaw, vx, north2, east2, yaw2, vx2):
        print ('pub tf tree')
        date = time.time()

        baseSensor = BaseSensor()
        baseSensor.x = east
        baseSensor.y = north
        baseSensor.yaw = pi/2 - yaw
        baseSensor.vx = vx
        baseSensor.vy = 0
        baseSensor.x_target = east2
        baseSensor.y_target = north2
        baseSensor.yaw_target = pi/2 - yaw2
        baseSensor.vx_target = vx2
        baseSensor.vy_target = 0

        self.sensor_pub.publish(baseSensor)

        print ('north: ', north, 'east: ', east, 'yaw: ', yaw)
        # east : x
        # north : y
        # yaw: yaw 
        self.br_boat_world.sendTransform((baseSensor.x ,baseSensor.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, baseSensor.yaw),
                     rospy.Time.now(),
                     "boat",
                     "world")

        self.br_lidar_boat.sendTransform((0, 0, 0.2),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "velodyne",
                     "boat")
        
        # self.br_lidar_boat2.sendTransform((0, 0, 0.2),
        #              tf.transformations.quaternion_from_euler(0, 0, 0),
        #              rospy.Time.now(),
        #              "lidar",
        #              "boat")
        
        self.br_camera_boat.sendTransform((0, 0, 0.1),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "camera",
                     "boat")

        print ('4: ', time.time()-date)


if __name__ == '__main__':
    rospy.init_node('boat_tf_broadcaster_interface')
    synchronizer = synchronizer()
    rospy.spin()
    