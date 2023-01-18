import rospy
import cv2
import cv2.aruco as aruco
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import quaternion_from_matrix

class ArucoEstimator():

    def __init__(self):
        rospy.init_node('aruco_estimator', anonymous=True)
        rospy.loginfo('Initialized aruco_estimator node')

        self.bridge = CvBridge()

        self.ARUCO_DICT = {
	        'DICT_4X4_50': aruco.DICT_4X4_50,
            'DICT_4X4_10': aruco.DICT_4X4_100,
            'DICT_4X4_250': aruco.DICT_4X4_250,
            'DICT_4X4_1000': aruco.DICT_4X4_1000,
            'DICT_5X5_50': aruco.DICT_5X5_50,
            'DICT_5X5_100': aruco.DICT_5X5_100,
            'DICT_5X5_250': aruco.DICT_5X5_250,
            'DICT_5X5_1000': aruco.DICT_5X5_1000,
            'DICT_6X6_50': aruco.DICT_6X6_50,
            'DICT_6X6_100': aruco.DICT_6X6_100,
            'DICT_6X6_250': aruco.DICT_6X6_250,
            'DICT_6X6_1000': aruco.DICT_6X6_1000,
            'DICT_7X7_50': aruco.DICT_7X7_50,
            'DICT_7X7_100': aruco.DICT_7X7_100,
            'DICT_7X7_250': aruco.DICT_7X7_250,
            'DICT_7X7_1000': aruco.DICT_7X7_1000,
            'DICT_ARUCO_ORIGINAL': aruco.DICT_ARUCO_ORIGINAL,
            'DICT_APRILTAG_16h5': aruco.DICT_APRILTAG_16h5,
            'DICT_APRILTAG_25h9': aruco.DICT_APRILTAG_25h9,
            'DICT_APRILTAG_36h10': aruco.DICT_APRILTAG_36h10,
            'DICT_APRILTAG_36h11': aruco.DICT_APRILTAG_36h11
        }

        self._read_config()
        self._get_calib_info()
        self.detector = aruco.ArucoDetector(self.dictionary, self.detector_params)
        self._setup_subscribers()


    def _read_config(self):
        self.camera_topic_prefix = rospy.get_param('~camera_topic_prefix')
        self.dictionary = aruco.getPredefinedDictionary(self.ARUCO_DICT.get(rospy.get_param('~dictionary_type')))
        self.detector_params = aruco.DetectorParameters()
        self.marker_length = rospy.get_param('~marker_length')
        self.topic_prefix = rospy.get_param('~topic_prefix', '/positioning')

    def _setup_subscribers(self):
        rospy.loginfo('Subscribing to {}image_raw'.format(self.camera_topic_prefix))
        self.image_sub = rospy.Subscriber(self.camera_topic_prefix + 'image_raw', Image, self._image_callback, queue_size=1)

    def _get_calib_info(self):
        rospy.loginfo('Getting camera matrix and distortion coeffs.')
        msg = rospy.wait_for_message(self.camera_topic_prefix+'camera_info', CameraInfo, timeout = 10)
        self.camera_matrix = np.reshape(msg.K, (3, 3))
        self.distortion_coeffs = msg.D
        rospy.loginfo('Got camera matrix and distortion coeffs.')


    def _image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, _, _ = self.detector.detectMarkers(gray)
        if len(corners) > 0:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.distortion_coeffs)
            aruco.drawDetectedMarkers(frame, corners)            
            cv2.drawFrameAxes(frame, self.camera_matrix, self.distortion_coeffs, rvecs, tvecs, self.marker_length / 2)

            rot_mat =  np.array([
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 1]],
                dtype=float
            )
            rot_mat[:3, :3], _ = cv2.Rodrigues(rvecs)
            quaternion = quaternion_from_matrix(rot_mat)

            pose = PoseStamped()
            pose.header.frame_id = 'camera_frame'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = tvecs[0][0][0]
            pose.pose.position.y = tvecs[0][0][1]
            pose.pose.position.z = tvecs[0][0][2]
            pose.pose.orientation = Quaternion(*quaternion)

            self.cam_pos_pub.publish(pose)

            cv2.putText(frame, str(f'{tvecs[0][0][0]:.{4}f}'+ '    ' +
                               f'{tvecs[0][0][1]:.{4}f}'    + '    ' +
                               f'{tvecs[0][0][2]:.{4}f}'    + '    '), (20, 70+20),
                               cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, str(f'{tvecs[0][0][0]:.{4}f}' + '    ' +
                               f'{tvecs[0][0][1]:.{4}f}'     + '    ' +
                               f'{tvecs[0][0][2]:.{4}f}' + '    '), (20, 70+20),
                               cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1, cv2.LINE_AA)

        else:
            cv2.putText(frame, 'NOT FOUND', (20, 30), cv2.FONT_HERSHEY_PLAIN,
                    1, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(frame, 'NOT FOUND', (20, 30), cv2.FONT_HERSHEY_PLAIN,
                    1, (0, 0, 0), 1, cv2.LINE_AA)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        except CvBridgeError as e:
            rospy.logerr(e)
            return


    def start(self):
        rospy.loginfo('Will publish to {}image_info'.format(self.camera_topic_prefix))
        self.image_pub = rospy.Publisher(self.camera_topic_prefix + 'image_info', Image, queue_size=1)
        rospy.loginfo('Will publish to {}/aruco/'.format(self.topic_prefix))
        self.cam_pos_pub = rospy.Publisher(self.topic_prefix+'/aruco/', PoseStamped, queue_size = 10)
        rospy.spin()