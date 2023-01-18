#!/usr/bin/env python3

import rospy
import math
import itertools
import numpy as np

from gazebo_msgs.msg import ModelStates
from mp_uav_landing_sim.msg import UWBRange

from uwb_interpolation.rmse_interpolation import RMSESplineInterpolator
from uwb_interpolation.sigma_interpolation import SigmaSplineInterpolator

class UWBSimulator():

    def __init__(self):
        self._read_config()
        self._create_publishers()

        self.tag_position = {}
        self.anchor_positions = {}

        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._get_positions_callback)
        rospy.loginfo('Subscribed to \'/gazebo/model_states\' publisher')

        self.sigma_interpolator = SigmaSplineInterpolator()
        self.rmse_initerpolator = RMSESplineInterpolator()

    def _read_config(self):
        self.update_rate = rospy.Rate(rospy.get_param('~update_rate', 10))
        self.tag = rospy.get_param('~tag', 'iris')
        self.anchors = rospy.get_param('~anchors', ['uwb_anch'])

    def _create_publishers(self):
        self.publishers = []

        for anchor in self.anchors:
            rospy.loginfo('Will publish to /uwb/tag/' + self.tag + '/anchor/' + anchor)
            pub = rospy.Publisher('/uwb/tag/' + self.tag + '/anchor/' + anchor, UWBRange, queue_size=10)
            self.publishers.append(pub)
        
    def _get_positions_callback(self, msg):
        names = msg.name
        poses = msg.pose

        tag_idx = names.index(self.tag)
        self.tag_position.update({self.tag : poses[tag_idx].position})

        for anchor in self.anchors:
            anchor_idx = names.index(anchor)
            self.anchor_positions.update({anchor : poses[anchor_idx].position})

    @staticmethod
    def _calculate_ground_truth_distance(tag_pos, anch_pos):
        return math.sqrt((tag_pos.x - anch_pos.x) ** 2 + (tag_pos.y - anch_pos.y) ** 2 + (tag_pos.z - anch_pos.z) ** 2)
        
    def _estimate_uwb_range(self, anchor_pos):
        tag_pos = next(iter(self.tag_position.items()))[1]

        ground_truth = self._calculate_ground_truth_distance(tag_pos, anchor_pos)
        if ground_truth > 20:
            return ground_truth, float('NaN')

        estimated = ground_truth + \
            self.rmse_initerpolator.interpolate(ground_truth) + \
                np.random.default_rng().normal(0, self.sigma_interpolator.interpolate(ground_truth))

        return ground_truth, estimated

    def start(self):
        rospy.init_node('uwb_simulator', anonymous=True)
        rospy.loginfo('Initialized uwb_simulator node')

        while not rospy.is_shutdown():
            pub = itertools.count(0)

            for anchor_pos in self.anchor_positions.values():
                msg = UWBRange()

                msg.header.stamp = rospy.Time.now()
                gt, est = self._estimate_uwb_range(anchor_pos)
                msg.status = 0 if math.isnan(est) else 1
                msg.estimated_range = est
                msg.ground_truth_range = gt

                #rospy.loginfo(msg)

                self.publishers[next(pub)].publish(msg)
                self.update_rate.sleep()

if __name__ == '__main__':
    try:
        UWBSimulator().start()
    except rospy.ROSInterruptException:
        pass