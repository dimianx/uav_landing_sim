#!/usr/bin/env python3

import rospy
import numpy as np
from math import log10
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool

class LandingController():

    def __init__(self):
        rospy.init_node('landing_controller', anonymous=True)
        rospy.loginfo('Initialized landing_controller node')

        self.dt = 1 / 150
        self.rate = rospy.Rate(1 / self.dt)

        self.relative_pos = [0, 0, 0]
        self.prev_error = [0, 0]
        self.integral = [0, 0]
        self.I_prev = [0, 0]

        self.lp_filter_estimate = [0, 0]
        self.lp_filter_prev_estimate = [0, 0]

        self.prev_timestamp = None
        self.timestamp = None
        self.target_lost = False
        self.land_ena = False
        self.time = 0

        self._read_config()
        self._setup_subscribers()   
        self._setup_publishers()

    def _read_config(self):
        self.pos_topic = rospy.get_param('~pose_topic', 'uwb_lqr_vanc')
        self.a = rospy.get_param('~a', 20)
        self.b = rospy.get_param('~b', -17)
        self.Vz0 = rospy.get_param('~Vz0', 1.5)
        self.alpha = rospy.get_param('alpha', 0.7)

    def _setup_subscribers(self):
        rospy.loginfo('Subscribing to {}'.format(self.pos_topic))
        self.pose_sub = rospy.Subscriber(self.pos_topic, PoseStamped, self._positioning_callback)

        rospy.loginfo('Subscribing to /landing/enabled')
        self.land_ena_sub = rospy.Subscriber('/landing/enabled', Bool, self._land_ena_callback)

    def _setup_publishers(self):
        rospy.loginfo('Will publish to /landing/setpoint_vel')
        self.vel_pub = rospy.Publisher('/landing/setpoint_vel', TwistStamped, queue_size = 10)

    def _positioning_callback(self, msg):
        self.timestamp = msg.header.stamp

        self.relative_pos[0] = msg.pose.position.x
        self.relative_pos[1] = msg.pose.position.y
        self.relative_pos[2] = msg.pose.position.z

    def _land_ena_callback(self, msg):
        self.land_ena = msg.data

    def _send_vel(self, vel):
        msg = TwistStamped()

        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = vel[0]
        msg.twist.linear.y = vel[1]
        msg.twist.linear.z = vel[2]

        self.vel_pub.publish(msg)

    def start(self):
        while not rospy.is_shutdown():
            if self.relative_pos == [0, 0, 0] or not self.land_ena:
                self.target_lost = False
                self._send_vel([0, 0, 0])
                self.rate.sleep()
                continue

            command = [0, 0, 0]
            error = np.array(self.relative_pos)

            stamp = self.timestamp
            if stamp == self.prev_timestamp:
                self.time += self.dt
                if self.time >= 1.:
                    self.target_lost = True
            else:
                self.time = 0.
                self.target_lost = False


            for i in range(2):
                self.lp_filter_estimate[i] = (self.alpha * self.lp_filter_prev_estimate[i]) + (1 - self.alpha) * \
                      (self.lp_filter_estimate[i] - self.lp_filter_prev_estimate[i]) / self.dt
                self.lp_filter_prev_estimate[i] = self.lp_filter_prev_estimate[i]

                P = np.exp(-0.199)*np.exp(-0.098*error[2]) * error[i]
                D = np.exp(-9.21) * np.exp(0.321 *error[2]) * self.lp_filter_estimate[i]
                
                self.integral[i] += ((self.prev_error[i] + error[i]) / 2) * self.dt
                if abs(error[0]) <= 0.5 and abs(error[1]) <= 0.5:
                    I = self.I_prev[i]
                else:
                    I = (np.exp(-1.878) * np.exp(-0.154 * error[2])) * self.integral[i]

                self.prev_error[i] = error[i]
                self.I_prev[i] = I
                command[i] = (P + I + D)          

            if abs(error[0]) <= 0.45 and abs(error[1]) <= 0.5:
                if abs(error[2]) <= 0.35:
                    command[2] = np.nan
                else:
                    x = max(1, abs(error[2] / rospy.get_param('/drone_fsm/takeoff_altitude')))
                    command[2] = -self.Vz0 * log10(1 + self.a * x + self.b * x ** 2)

            if self.target_lost:
                if self.time >= 3:
                    command[2] = np.inf
                    self.time = 0
                    self.relative_pos = [0, 0, 0]
                    self.error = self.relative_pos
                elif abs(error[2]) < 7:
                    command[2] = 5
                    command[0] = 0
                    command[1] = 0

            self.prev_timestamp = stamp
            self._send_vel(command)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        LandingController().start()
    except rospy.ROSInterruptException:
        pass
