#!/usr/bin/env python3

import rospy
import math
import random
import numpy as np

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from mp_uav_landing_sim.msg import Intensity
from tf.transformations import euler_from_quaternion

from mp_uav_landing_sim.msg import Intensity

class InfraOpticalSimulator():

    def __init__(self):
        rospy.init_node('infraoptical_simulator', anonymous=True)
        rospy.loginfo('Initialized infraoptical_simulator node')

        self.tag_position = {}
        self.diode_pairs_positions = {}
        self.diode_pairs = ['diode0', 'diode1', 'diode2', 'diode3']

        self._read_config()
        self._create_publishers()

        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._get_positions_callback)
        rospy.loginfo('Subscribed to \'/gazebo/model_states\' publisher')

    def _read_config(self):
        self.update_rate = rospy.Rate(rospy.get_param('~update_rate', 7182))
        self.tag = rospy.get_param('~tag', 'iris')
        self.ugv = rospy.get_param('~ugv', 'husky')
        self.I0 = rospy.get_param('~I0', 0.5)
        self.half_theta_receiver = rospy.get_param('~half_theta_receiver', 1.57)
        self.half_theta_transmitter = rospy.get_param('~half_theta_transmitter', 1.57)
        self.gamma = rospy.get_param('~gamma', 3.14 / 10)
        self.diode_pairs_coordinates = rospy.get_param('~diode_pairs_coordinates', [])

    def _create_publishers(self):
        self.publishers = []

        for diode_pair in self.diode_pairs:
            rospy.loginfo('Will publish to /infraoptical/tag/' + self.tag + '/diode_pair/' + diode_pair)
            pub = rospy.Publisher('/infraoptical/tag/' + self.tag + '/diode_pair/' + diode_pair, Intensity, queue_size=10)
            self.publishers.append(pub)

    def _get_positions_callback(self, msg):
        names = msg.name
        poses = msg.pose

        tag_idx = names.index(self.tag)
        ugv_idx = names.index(self.ugv)
        self.tag_position.update({self.tag : poses[tag_idx].position})

        pair_idx = 0

        for diode_pair in self.diode_pairs:
            ugv_pos = np.array([
                poses[ugv_idx].position.x,
                poses[ugv_idx].position.y,
                poses[ugv_idx].position.z
            ])

            self.diode_pairs_positions.update({diode_pair : np.add(ugv_pos, np.array(self.diode_pairs_coordinates[pair_idx]))})
            pair_idx += 1

    def _get_relative_rotation(self):
        model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        state = model_state(self.tag, self.ugv)

        quaternion = state.pose.orientation
        euler = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        return euler

    def _add_noise(self, I):
        return I + random.randint(-1, 1) * (I * random.randint(10, 12) / 10000)

    def _estimate_intensity(self, tag_pose, pair_pos0, pair_pos1, phi, omega):
        if phi <= -0.1 or omega <= -0.1:
            return float('NaN'), float('NaN'), float('NaN'), float('NaN')

        R1 = math.sqrt((tag_pose[0] - pair_pos1[0]) ** 2 + (tag_pose[1] - pair_pos1[1]) ** 2)
        D = abs(tag_pose[0] - pair_pos0[0])
        if -tag_pose[0] + pair_pos0[0] <= 0.15:
            return float('NaN'), float('NaN'), float('NaN'), float('NaN')

        alpha = math.atan2(math.sqrt((tag_pose[1] - pair_pos1[1]) ** 2), D)        
        if tag_pose[1] > pair_pos1[1]:
            alpha = math.pi - alpha

        R2 =  math.sqrt((tag_pose[0] - pair_pos0[0]) ** 2 + (tag_pose[1] - pair_pos0[1]) ** 2)

        beta = math.atan2(math.sqrt((tag_pose[1] - pair_pos0[1]) ** 2), D)
        if tag_pose[1] <= pair_pos0[1]:
            beta = math.pi - beta

        n = -math.log(2) / math.log(math.cos(self.half_theta_receiver))
        m = -math.log(2) / math.log(math.cos(self.half_theta_transmitter))

        I1 = self.I0 / (R1 ** 2) * (math.cos(phi) ** n) * (math.cos(alpha + self.gamma) ** m)
        I2 = self.I0 / (R1 ** 2) * (math.cos(phi) ** n) * (math.cos(alpha - self.gamma) ** m)

        I3 = self.I0 / (R2 ** 2) * (math.cos(omega) ** n) * (math.cos(beta + self.gamma) ** m)
        I4 = self.I0 / (R2 ** 2) * (math.cos(omega) ** n) * (math.cos(beta - self.gamma) ** m)

        if isinstance(I1, complex): I1 = float(I1.real)
        if isinstance(I2, complex): I2 = float(I2.real)
        if isinstance(I3, complex): I3 = float(I3.real)
        if isinstance(I4, complex): I4 = float(I4.real)

        return I1, I2, I3, I4
    
    def start(self):
        rospy.sleep(8)
        while not rospy.is_shutdown():
            values = list(self.diode_pairs_positions.values())
            tag_pos = next(iter(self.tag_position.items()))[1]
        
            msg0 = Intensity()
            msg1 = Intensity()
            msg2 = Intensity()
            msg3 = Intensity()

            phi0 = self._get_relative_rotation()[2]
            omega0 = math.pi / 2 - phi0 

            I1, I2, I3, I4 = self._estimate_intensity(
                [tag_pos.x, tag_pos.y],
                [values[0][0], values[0][1]],
                [values[1][0], values[1][1]],
                phi0,
                omega0       
            )

            msg0.status = 0 if math.isnan(I1) else 1
            msg0.estimated_intensity0 = self._add_noise(I1)
            msg0.estimated_intensity1 = self._add_noise(I2)
            msg0.ground_truth_intensity0 = I1
            msg0.ground_truth_intensity1 = I2

            msg1.status = 0 if math.isnan(I3) else 1
            msg1.estimated_intensity0 = self._add_noise(I3)
            msg1.estimated_intensity1 = self._add_noise(I4)
            msg1.ground_truth_intensity0 = I3
            msg1.ground_truth_intensity1 = I4               

            phi1 = self._get_relative_rotation()[1] 
            omega1 = math.pi / 2 - phi1
            
            I5, I6, I7, I8 = self._estimate_intensity(
                [tag_pos.x, tag_pos.z],
                [values[2][0], values[2][2]],
                [values[3][0], values[3][2]],
                phi1,
                omega1
            )

            msg2.status = 0 if math.isnan(I5) else 1
            msg2.estimated_intensity0 = self._add_noise(I5)
            msg2.estimated_intensity1 = self._add_noise(I6)
            msg2.ground_truth_intensity0 = I5
            msg2.ground_truth_intensity1 = I6

            msg3.status = 0 if math.isnan(I7) else 1
            msg3.estimated_intensity0 = self._add_noise(I7)
            msg3.estimated_intensity1 = self._add_noise(I8)
            msg3.ground_truth_intensity0 = I7
            msg3.ground_truth_intensity1 = I8

            msg0.header.stamp = rospy.Time.now() 
            msg1.header.stamp = rospy.Time.now() 
            msg2.header.stamp = rospy.Time.now() 
            msg3.header.stamp = rospy.Time.now() 

            self.publishers[0].publish(msg0)
            self.publishers[1].publish(msg1)
            self.publishers[2].publish(msg2)
            self.publishers[3].publish(msg3)

            self.update_rate.sleep()

if __name__ == '__main__':
    try:
        InfraOpticalSimulator().start()
    except (rospy.ROSInterruptException, StopIteration):
        pass