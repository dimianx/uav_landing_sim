#!/usr/bin/env python3

import rospy
import math

from mp_uav_landing_sim.msg import Intensity
from geometry_msgs.msg import PoseStamped, Quaternion

class InfraOpticalEstimator():

    def __init__(self):
        rospy.init_node('infra_optical_estimator', anonymous=True)
        rospy.loginfo('Initialized infra_optical_estimator node')

        self.intensity_subs = []
        self.intensities = {}

        self._read_config()
        self._setup_subscribers()
        self._setup_publishers()
        self._create_intensities_dict()

    def _read_config(self):
        self.distance_x = rospy.get_param('~distance_x', 0.1)
        self.distance_z = rospy.get_param('~distance_z', 0.1)
        self.gamma = rospy.get_param('~gamma', 0.314)
        self.half_theta_receiver = rospy.get_param('~half_theta_receiver', 1.046)
        self.half_theta_transmitter = rospy.get_param('~half_theta_transmitter', 1.57)
        self.update_rate = rospy.Rate(rospy.get_param('~update_rate', 100))

        self.intensity_topics = rospy.get_param('~intensity_topics', [])

    def _setup_subscribers(self):
        rospy.loginfo('Subscribing to IR intensities topics')

        for topic in self.intensity_topics:
            self.intensity_subs.append(rospy.Subscriber(topic, Intensity, self._positioning_callback, topic))

    def _setup_publishers(self):
        rospy.loginfo('Will publish to /positioning/infra_optical')
        self.tag_pos_pub = rospy.Publisher('/positioning/infra_optical', PoseStamped, queue_size=10)
    
    def _create_intensities_dict(self):
        for topic in self.intensity_topics:
            self.intensities.update({topic : [float('NaN'), float('NaN')]})

    def _calculate_positions(self):
        intensities = list(self.intensities.values())
        X = [0] * 3

        m = -math.log(2) / math.log(math.cos(self.half_theta_transmitter))

        tga = (1 - (intensities[0][0] / intensities[0][1]) ** (1 / m)) / (1 + (intensities[0][0] / intensities[0][1]) ** (1 / m)) * (1 / math.tan(self.gamma))
        tgb = (1 - (intensities[1][0] / intensities[1][1]) ** (1 / m)) / (1 + (intensities[1][0] / intensities[1][1]) ** (1 / m)) * (1 / math.tan(self.gamma))
        X[1] = self.distance_x / 2 - self.distance_x * tga / (tgb + tga)
        X[0] = self.distance_x / (tgb + tga)

        tga = (1 - (intensities[2][0] / intensities[2][1]) ** (1 / m)) / (1 + (intensities[2][0] / intensities[2][1]) ** (1 / m)) * (1 / math.tan(self.gamma))
        tgb = (1 - (intensities[3][0] / intensities[3][1]) ** (1 / m)) / (1 + (intensities[3][0] / intensities[3][1]) ** (1 / m)) * (1 / math.tan(self.gamma))
        X[2] = self.distance_z / 2 - self.distance_z * tga / (tgb + tga)

        return X

    def _positioning_callback(self, msg, args):
        if len(self.intensities) < 4:
            raise Exception('The minimum number of anchors must be equal to 4')

        self.intensities.update({args : [msg.estimated_intensity0, msg.estimated_intensity1]})
        if any(math.isnan(intensity[0]) for intensity in  self.intensities.values()):
            rospy.logwarn_once('Out of ranges')
            return
        
        if args == list(self.intensities.keys())[-1]:
            pose = PoseStamped()
            pose.header.frame_id = 'ir_anchor_frame'
            pose.header.stamp = rospy.Time.now()
            X = self._calculate_positions()
            pose.pose.position.x = X[0] - 0.8
            pose.pose.position.y = -X[1]
            pose.pose.position.z = X[2] - 0.7
            pose.pose.orientation = Quaternion(0,0,0,0)

            self.tag_pos_pub.publish(pose)

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        InfraOpticalEstimator().start()
    except rospy.ROSInterruptException:
        pass