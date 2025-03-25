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
import cv2
import os
import aruco
import random
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_matrix
from env_disturbances import env_disturbances

class ArucoEstimator():

    def __init__(self):
        rospy.init_node('aruco_estimator', anonymous=True)
        rospy.loginfo('Initialized aruco_estimator node')

        self.detector = aruco.FractalDetector()
        self.detector.setConfiguration(aruco.FractalMarkerSet.FRACTAL_5L_6)
        self.bridge = CvBridge()

        self._read_config()
        self._load_camera_params()
        self._setup_subscribers()
        self._setup_publishers()

    def _read_config(self):
        self.camera_topic_prefix = rospy.get_param('~camera_topic_prefix')
        self.camera_parameters = rospy.get_param('~camera_parameters_file')
        self.marker_size = rospy.get_param('~marker_size')
        self.ena_disturbances = rospy.get_param('~ena_disturbances', False)
        self.brightness = rospy.get_param('~brightness', 1)
        self.haze_coeff = rospy.get_param('~haze_coeff', 0)
        self.rain_value = rospy.get_param('~rain_value', 30000)

    def _setup_subscribers(self):
        rospy.loginfo('Subscribing to {}image_raw'.format(self.camera_topic_prefix))
        self.image_sub = rospy.Subscriber(self.camera_topic_prefix + 'image_raw', Image, self._image_callback, queue_size=1)

    def _setup_publishers(self):
        rospy.loginfo('Will publish to {}image_info'.format(self.camera_topic_prefix))
        self.image_pub = rospy.Publisher(self.camera_topic_prefix + 'image_info', Image, queue_size=1)
        rospy.loginfo('Will publish to /positioning/aruco')
        self.cam_pos_pub = rospy.Publisher('/positioning/aruco/', PoseStamped, queue_size = 10)

    def _load_camera_params(self):
        rospy.loginfo('Loading camera parameters')
        
        camparam = aruco.CameraParameters()
        camparam.readFromXMLFile(filePath=os.path.join(
            os.path.dirname(__file__),
            '..', '..', 'config', 'camera_calibration', 
            self.camera_parameters)
        )

        if camparam.isValid():
            self.detector.setParams(camparam, self.marker_size)
        else:
            raise Exception('Invalid camera parameters!')

    def _image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')#
            
            if self.ena_disturbances:
                if not self.haze_coeff == 0:
                    frame = env_disturbances.add_haze(frame, self.haze_coeff)

                if not self.rain_value == 0:
                    frame = env_disturbances.add_streaks(frame, noise_value=self.rain_value, angle=random.randint(-30,30))

                if not self.brightness == 1:
                    frame = env_disturbances.adjust_brightness(frame, self.brightness)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detected = self.detector.detect(gray)

        if detected:
            rospy.loginfo_once('Marker found')
            self.detector.drawMarkers(frame)
            
            markers = self.detector.getMarkers()
            for marker in markers:
                marker.draw(frame, np.array([255, 255, 255]), 2)

            self.detector.draw2d(frame)

            if self.detector.poseEstimation():
                tvec = self.detector.getTvec()
                rvec = self.detector.getRvec()

                self.detector.draw3d(frame)

            rot_mat =  np.array([
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 1]],
                dtype=float
            )
            rot_mat[:3, :3], _ = cv2.Rodrigues(rvec)
            quaternion = quaternion_from_matrix(rot_mat)

            pose = PoseStamped()
            pose.header.frame_id = 'camera_frame'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = tvec[0][0]
            pose.pose.position.y = -tvec[1][0]
            pose.pose.position.z = tvec[2][0]
            pose.pose.orientation = Quaternion(*quaternion)

            self.cam_pos_pub.publish(pose)

            cv2.putText(frame, str(f'{tvec[0][0]:.{4}f}'+ '    ' +
                               f'{-tvec[1][0]:.{4}f}'    + '    ' +
                               f'{tvec[2][0]:.{4}f}'    + '    '), (20, 70+20),
                               cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, str(f'{tvec[0][0]:.{4}f}' + '    ' +
                               f'{-tvec[1][0]:.{4}f}'     + '    ' +
                               f'{tvec[2][0]:.{4}f}' + '    '), (20, 70+20),
                               cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1, cv2.LINE_AA)

        else:
            cv2.putText(frame, 'NOT FOUND', (20, 30), cv2.FONT_HERSHEY_PLAIN,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, 'NOT FOUND', (20, 30), cv2.FONT_HERSHEY_PLAIN,
                    1, (0, 0, 0), 1, cv2.LINE_AA)
            rospy.logwarn_once('Marker not found.')

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        except CvBridgeError as e:
            rospy.logerr(e)
            return


    def start(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        ArucoEstimator().start()
    except rospy.ROSInterruptException:
        pass