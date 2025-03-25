#!/usr/bin/env python3

 # 
 # This file is part of the uav_landing_sim distribution (https://github.com/dimianx/uav_landing_sim).
 # Copyright (c) 2024-2025 Dmitry Anikin <dmitry.anikin@proton.me>.
 # 
 # This program is free software: you can redistribute it and/or modify  
 # it under the terms of the GNU General Public License as published by  
 # the Free Software Foundation, version 3.
 #
 # This program is distributed in the hope that it will be useful, but 
 # WITHOUT ANY WARRANTY; without even the implied warranty of 
 # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 # General Public License for more details.
 #
 # You should have received a copy of the GNU General Public License 
 # along with this program. If not, see <http://www.gnu.org/licenses/>.
 #

import rospy
import numpy as np

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from numpy.linalg import norm
from math import atan2, sin, cos

class UGVController():

    def __init__(self):
        rospy.init_node('ugv_controller', anonymous=True)
        rospy.loginfo('Initialized ugv_controller node')

        self.rate = rospy.Rate(20)

        self.position = [0] * 2
        self.theta = 0.0

        self.odom_sub = rospy.Subscriber('/husky/odometry/filtered', Odometry, self._odom_callback)
        self.vel_pub = rospy.Publisher('/husky/cmd_vel', Twist, queue_size=10)

        self._read_config()

    def _read_config(self):
        self.wgain = rospy.get_param('~wgain', 0)
        self.vconst = rospy.get_param('~vconst', 0)
        self.waypoints = rospy.get_param('~waypoints', [])

    def _odom_callback(self, msg:Odometry):
        self.position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ]

        angles = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        self.theta = angles[2]

    def start(self):
        idx = 0

        while not rospy.is_shutdown():
            wp = self.waypoints[idx]

            msg = Twist()

            distance = norm(np.subtract(np.array(self.position), np.array(wp)))
            v = 0
            w = 0
            
            if distance > 0.15:
                v = self.vconst
                dy = wp[1] - self.position[1]
                dx = wp[0] - self.position[0]
                yaw = atan2(dy, dx)
                u = yaw - self.theta
                bound = atan2(sin(u), cos(u))
                w = min(0.5, max(-0.5, self.wgain * bound))
            else:
                idx = (idx + 1) % len(self.waypoints)

            msg.linear.x = v
            msg.angular.z = w
            self.vel_pub.publish(msg)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        UGVController().start()
    except rospy.ROSInterruptException:
        pass