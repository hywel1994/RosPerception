#!/usr/bin/env python  
import roslib
import rospy

import tf
import time
import numpy as np

from sensor_msgs.msg import Imu, NavSatFix
import message_filters


class synchronizer:
    def __init__(self):
        # self.pub_Image = rospy.Publisher('image_raw_sync', SesnorImage, queue_size=1)
        # self.pub_Cam_Info = rospy.Publisher('camera_info_sync', CameraInfo, queue_size=1)
        # self.pub_Lidar = rospy.Publisher('rslidar_points_sync', PointCloud2, queue_size=1)
        self.imuInput = message_filters.Subscriber("/imu", Imu)
        self.gpsInput = message_filters.Subscriber('/gps/fix', NavSatFix)

        self.ts = message_filters.TimeSynchronizer([self.imuInput
                                                    , self.gpsInput
                                                    ], 10)
        self.ts.registerCallback(self.general_callback)
        self.br_boat_world = tf.TransformBroadcaster()
        self.br_lidar_boat = tf.TransformBroadcaster()
        self.br_lidar_boat2 = tf.TransformBroadcaster()
        self.br_camera_boat = tf.TransformBroadcaster()
        self._imu_raw = Imu()
        self._gps_raw = NavSatFix()
        

        
    def general_callback(self, imu_raw, gps_raw):
        print ('pub tf tree')
        self._imu_raw = imu_raw
        self._gps_raw = gps_raw
        # print (msg)
        date = time.time()

        quaternion = (
            self._imu_raw.orientation.x,
            self._imu_raw.orientation.y,
            self._imu_raw.orientation.z,
            self._imu_raw.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        self.br_boat_world.sendTransform((self._gps_raw.longitude, self._gps_raw.latitude, 0),
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