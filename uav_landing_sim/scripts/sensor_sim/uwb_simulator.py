#!/usr/bin/env python3

 # 
 # This file is part of the uav_landing_sim distribution (https://github.com/dimianx/uav_landing_sim).
 # Copyright (c) 2025 Dmitry Anikin.
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
import math
import itertools
import numpy as np

from gazebo_msgs.msg import ModelStates
from uav_landing_sim.msg import UWBRange

from uwb_interpolation.rmse_interpolation import RMSESplineInterpolator
from uwb_interpolation.sigma_interpolation import SigmaSplineInterpolator

class UWBSimulator():

    def __init__(self):
        rospy.init_node('uwb_simulator', anonymous=True)
        rospy.loginfo('Initialized uwb_simulator node')

        self.tag_position = {}
        self.anchor_positions = {}
        self.anchors = ['anchor0', 'anchor1', 'anchor2', 'anchor3']

        self._read_config()
        self._create_publishers()

        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._get_positions_callback)
        rospy.loginfo('Subscribed to \'/gazebo/model_states\' publisher')

        self.sigma_interpolator = SigmaSplineInterpolator()
        self.rmse_initerpolator = RMSESplineInterpolator()

    def _read_config(self):
        self.update_rate = rospy.Rate(rospy.get_param('~update_rate', 10))
        self.tag = rospy.get_param('~tag', 'iris')
        self.ugv = rospy.get_param('~ugv', 'husky')
        self.anchor_coordinates = rospy.get_param('~anchor_coordinates', [])

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
        ugv_idx = names.index(self.ugv)
        self.tag_position.update({self.tag : poses[tag_idx].position})

        anch_idx = 0

        for anchor in self.anchors:
            ugv_pos = np.array([
                poses[ugv_idx].position.x,
                poses[ugv_idx].position.y,
                poses[ugv_idx].position.z
            ])

            self.anchor_positions.update({anchor : np.add(ugv_pos, np.array(self.anchor_coordinates[anch_idx]))})
            anch_idx += 1

    @staticmethod
    def _calculate_ground_truth_distance(tag_pos, anch_pos):
        return math.sqrt((tag_pos.x - anch_pos[0]) ** 2 + (tag_pos.y - anch_pos[1]) ** 2 + (tag_pos.z - anch_pos[2]) ** 2)
        
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
        while not rospy.is_shutdown():
            pub = itertools.count(0)

            for anchor_pos in self.anchor_positions.values():
                msg = UWBRange()

                msg.header.stamp = rospy.Time.now()
                gt, est = self._estimate_uwb_range(anchor_pos)
                msg.status = 0 if math.isnan(est) else 1
                msg.estimated_range = est
                msg.ground_truth_range = gt

                self.publishers[next(pub)].publish(msg)
                self.update_rate.sleep()

if __name__ == '__main__':
    try:
        UWBSimulator().start()
    except rospy.ROSInterruptException:
        pass