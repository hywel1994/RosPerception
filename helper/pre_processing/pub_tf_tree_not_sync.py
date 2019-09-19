#!/usr/bin/env python  
import roslib
import rospy

import tf
import time
import numpy as np

from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Vector3Stamped
from helper.msg import BaseSensor 
import message_filters

from math import pi, sin, cos, sqrt

def d2r(d):
    return d/180.0*pi

class synchronizer:
    def __init__(self, rate=10):
        # self.pub_Image = rospy.Publisher('image_raw_sync', SesnorImage, queue_size=1)
        # self.pub_Cam_Info = rospy.Publisher('camera_info_sync', CameraInfo, queue_size=1)
        # self.pub_Lidar = rospy.Publisher('rslidar_points_sync', PointCloud2, queue_size=1)
        self.imuInput = rospy.Subscriber("/self/usv/imu", Imu,  self.imuCallback, queue_size=2)
        self.gpsInput = rospy.Subscriber('/self/usv/gps', NavSatFix, self.gpsCallback, queue_size=2)
        self.gpsVeloInput = rospy.Subscriber('/self/usv/gps/fix_velocity', Vector3Stamped, self.gpsVeloCallback, queue_size=2)

        self.targetGpsInput = rospy.Subscriber('/target/usv_2/gps', NavSatFix, self.targetGpsCallback, queue_size=2)
        self.targetGpsVeloInput = rospy.Subscriber('/target/usv_2/gps/fix_velocity', Vector3Stamped, self.targetGpsVeloCallback, queue_size=2)

        self.sensor_pub = rospy.Publisher('/base/sensor', BaseSensor, queue_size=1)
        # self.ts = message_filters.TimeSynchronizer([self.imuInput
        #                                             , self.gpsInput
        #                                             ], 10)
        # self.ts.registerCallback(self.general_callback)
        self.br_boat_world = tf.TransformBroadcaster()
        self.br_lidar_boat = tf.TransformBroadcaster()
        self.br_lidar_boat2 = tf.TransformBroadcaster()
        self.br_camera_boat = tf.TransformBroadcaster()
        
        self.gps_velo_msg = Vector3Stamped()
        self.target_gps_msg = NavSatFix()
        self.target_gps_velo_msg = Vector3Stamped()

        self.imuflag = False
        self.gpsflag = False

        self.lat = d2r(30.06022459407145675)
        self.lon = d2r(120.173913575780311191)

        rate = rospy.Rate(rate)

        while not rospy.is_shutdown():
            if self.imuflag and self.gpsflag:
                print ('process')
                self.general_callback(self.imu_msg, self.gps_msg, self.gps_velo_msg, self.target_gps_msg, self.target_gps_velo_msg)
                self.imuflag = False
                self.gpsflag = False

            rate.sleep()
    
    def w84_calc_ne(self, lat2, lon2):
        lat1,lon1 = self.lat,self.lon
        lat2,lon2 = d2r(lat2),d2r(lon2)
        d_lat = lat2-lat1
        d_lon = lon2-lon1

        a = 6378137.0
        e_2 = 6.69437999014e-3
        r1 = a*(1-e_2)/(1-e_2*(sin(lat1))**2)**1.5
        r2 = a/sqrt(1-e_2*(sin(lat1))**2)

        north = r1*d_lat
        east = r2*cos(lat1)*d_lon
        return north,east

            
    def imuCallback(self, imu_msg):
        self.imu_msg = imu_msg
        self.imuflag = True
    
    def gpsCallback(self, gps_msg):
        self.gps_msg = gps_msg
        self.gpsflag = True
    
    def gpsVeloCallback(self, gps_velo_msg):
        self.gps_velo_msg = gps_velo_msg
    
    def targetGpsCallback(self, gps_msg):
        self.target_gps_msg = gps_msg
    
    def targetGpsVeloCallback(self, gps_velo_msg):
        self.target_gps_velo_msg = gps_velo_msg
        
    def general_callback(self, imu_raw, gps_raw, gps_velo_raw, gps_raw2, gps_velo_raw2):
        print ('pub tf tree')
        # self._imu_raw = imu_raw
        # self._gps_raw = gps_raw
        # print (msg)
        date = time.time()

        quaternion = (
            imu_raw.orientation.x,
            imu_raw.orientation.y,
            imu_raw.orientation.z,
            imu_raw.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        north, east = self.w84_calc_ne(gps_raw.latitude, gps_raw.longitude)

        north2, east2 = self.w84_calc_ne(gps_raw2.latitude, gps_raw2.longitude)
        # vx2 = -gps_velo_raw2.vector.y
        # vy2 = gps_velo_raw2.vector.x

        baseSensor = BaseSensor()
        baseSensor.x = east
        baseSensor.y = north
        baseSensor.yaw = yaw
        baseSensor.vx = -gps_velo_raw.vector.y
        baseSensor.vy = gps_velo_raw.vector.x
        baseSensor.x_target = east2
        baseSensor.y_target = north2
        baseSensor.vx_target = -gps_velo_raw2.vector.y
        baseSensor.vy_target = gps_velo_raw2.vector.x

        self.sensor_pub.publish(baseSensor)

        print ('north: ', north, 'east: ', east, 'yaw: ', yaw)
        # east : x
        # north : y
        # yaw: -yaw 
        self.br_boat_world.sendTransform((east,north, 0),
                     tf.transformations.quaternion_from_euler(0, 0, yaw),
                     rospy.Time.now(),
                     "boat",
                     "world")

        self.br_lidar_boat2.sendTransform((0, 0, 0.2),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "velodyne",
                     "boat")
        
        self.br_lidar_boat.sendTransform((0, 0, 0.2),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "lidar",
                     "boat")
        
        self.br_camera_boat.sendTransform((0, 0, 0.1),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "camera",
                     "boat")

        print ('4: ', time.time()-date)


if __name__ == '__main__':
    rospy.init_node('boat_tf_broadcaster')
    synchronizer = synchronizer()
    rospy.spin()