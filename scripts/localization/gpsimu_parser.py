#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
import tf
import math

class GpsImuParser:
    def __init__(self):
        rospy.init_node("gpsimu_parser", anonymous=True)

        rospy.Subscriber("/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)

        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.is_gps, self.is_imu = False, False

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.is_gps and self.is_imu:
                odom = Odometry()
                odom.header.frame_id = "map"
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                q = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
                odom.pose.pose.orientation.x = q[0]
                odom.pose.pose.orientation.y = q[1]
                odom.pose.pose.orientation.z = q[2]
                odom.pose.pose.orientation.w = q[3]
                self.odom_pub.publish(odom)
            rate.sleep()

    def gps_callback(self, msg):
        # TODO: 실제 변환 (위경도 → x,y)
        self.x, self.y = msg.latitude, msg.longitude
        self.is_gps = True

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.is_imu = True


if __name__ == "__main__":
    try:
        GpsImuParser()
    except rospy.ROSInterruptException:
        pass
