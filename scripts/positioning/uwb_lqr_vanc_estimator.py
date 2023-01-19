#!/usr/bin/env python3

import rospy
import math
import numpy as np

from mp_uav_landing_sim.msg import UWBRange
from geometry_msgs.msg import PoseStamped, Quaternion 

class LQRVANCEstimator():

    def __init__(self):
        rospy.init_node('uwb_lqr_vanc_estimator', anonymous=True)
        rospy.loginfo('Initialized uwb_lqr_vanc_estimator node')

        self.uwb_subs = []
        self.distances = {}
        # Virtual anchor's position
        self.vanc_position = [20, 20, 20]
        self.iter_num = 10

        self._read_config()
        self._setup_subscribers()
        self._setup_publishers()
        self._create_distances_dict()

        self.anchor_positions.append(self.vanc_position)

    def _read_config(self):
        self.anchor_positions = rospy.get_param('~anchor_positions', [])
        self.uwb_topics = rospy.get_param('~uwb_topics', [])


    def _setup_subscribers(self):
        rospy.loginfo('Subscribing to uwb ranges topics')

        for topic in self.uwb_topics:
            self.uwb_subs.append(rospy.Subscriber(topic, UWBRange, self._positioning_callback, topic))

    def _setup_publishers(self):
        rospy.loginfo('Will publish to /positioning/uwb_lqr_vanc')
        self.tag_pos_pub = rospy.Publisher('/positioning/uwb_lqr_vanc', PoseStamped, queue_size=10)
    
    def _create_distances_dict(self):
        for topic in self.uwb_topics:
            self.distances.update({topic : float('NaN')})

        # Virutal anchor's distance
        self.distances.update({'vanc' : 0.0})

    def _calculate_positions(self):
        for i in range(self.iter_num):
            A = np.c_[np.array([1] * len(self.anchor_positions)).transpose(), -2 *np.array(self.anchor_positions)]
            B  = np.sum(-np.square(self.anchor_positions), axis = 1)
            B  = np.add(np.square(list(self.distances.values())), B)
            X = np.linalg.inv(np.matmul(A.transpose(), A))
            X = np.matmul(X, A.transpose())
            X = np.matmul(X, B)

            self.anchor_positions[-1] = list(X[1:])

        return list(X[1:])

    def _positioning_callback(self, msg, args):
        if len(self.distances) < 4 or len(self.anchor_positions) < 4:
            rospy.logerr('The minimum number of anchors must be >= 4')
            return

        if len(self.distances) != len(self.anchor_positions):
            rospy.loginfo(self.distances)
            rospy.loginfo(self.anchor_positions)
            rospy.logerr('The number of distance topics and anchor points is not equal')
            return

        self.distances.update({args : msg.estimated_range})
        if any(math.isnan(distance) for distance in  self.distances.values()):
            rospy.logwarn_once('Out of ranges')
            return

        pose = PoseStamped()
        pose.header.frame_id = 'uwb_tag_frame'
        pose.header.stamp = rospy.Time.now()
        X = self._calculate_positions()
        pose.pose.position.x = -X[0]
        pose.pose.position.y = -X[1]
        pose.pose.position.z = -X[2]
        pose.pose.orientation = Quaternion(0,0,0,0)
            
        rospy.loginfo(pose)
        self.tag_pos_pub.publish(pose)

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        LQRVANCEstimator().start()
    except rospy.ROSInterruptException:
        pass