import rospy

import numpy as np

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import NavSatFix

from tf.transformations import quaternion_from_euler

class DroneController():

    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)

        self.state = State()
        self.local_position = PoseStamped()
        self.global_position = NavSatFix()
        self.rate = rospy.Rate(20)

        self.prev_request_time = rospy.Time.now()
        
        self._setup_services()
        self._setup_subscribers()
        self._setup_publishers()

    def _setup_services(self):
        rospy.loginfo('Waiting for mavros services')

        try:
            rospy.wait_for_service('/mavros/set_mode', 30)
            rospy.wait_for_service('/mavros/cmd/arming', 30)

        except rospy.ROSException:
            rospy.logerr('Failed to connect to services')

        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    def _setup_subscribers(self):
        rospy.loginfo('Subscribing to mavros publishers')

        self.state_sub = rospy.Subscriber('/mavros/state', State, self._state_callback)
        self.local_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_position_callback)
        self.global_pose_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self._global_position_callback)

    def _setup_publishers(self):
        self.pos_setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped)

    def _state_callback(self, msg):
        self.state = msg

    def _local_position_callback(self, msg):
        self.local_position = msg

    def _global_position_callback(self, msg):
        self.global_position = msg

    def _set_arm(self, arm):
        if self.state.armed == arm:
            rospy.loginfo('Drone is already {st}'.format(st = 'armed' if arm == True else 'disarmed'))
        else:
            try:
                if rospy.Time.now() - self.prev_request_time > rospy.Duration(5.0):
                    result = self.set_arming_srv(arm)

                    if result.success:
                        rospy.loginfo("Setting FCU arm: {}".format(arm))
                    else:
                        rospy.logerr('Failed to set arm')
                    
                    self.prev_request_time = rospy.Time.now()
            except rospy.ServiceException as e:
                rospy.logerr(e)

    def _set_mode(self, mode):
        if self.state.mode == mode:
            rospy.loginfo('The mode is already set!')
        else:
            try:
                if rospy.Time.now() - self.prev_request_time > rospy.Duration(5.0):
                    result = self.set_mode_srv(0, mode)

                    if result.mode_sent:
                        rospy.loginfo("Set FCU mode: {0}".format(mode))
                    else:
                        rospy.logerr('Failed to send mode command')

                    self.prev_request_time = rospy.Time.now()
            except rospy.ServiceException as e:
                rospy.logerr(e)

    
    def _reach_local_position(self, x, y, z, yaw = np.pi / 2):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = Quaternion(*quaternion)

        self.pos_setpoint_pub.publish(msg)

    def _is_at_local_position(self, x, y, z, offset):
        desired = np.array((x, y, z))
        local_pos = np.array((
            self.local_position.pose.position.x,
            self.local_position.pose.position.y,
            self.local_position.pose.position.z
        ))

        return np.linalg.norm(desired - local_pos) < offset

    def start(self):
        if self.state.mode == self.state.MODE_PX4_OFFBOARD and self.state.armed():
            rospy.loginfo('The controller is already started!')
            return

        rospy.loginfo('Starting drone controller')

        rospy.loginfo('Waiting for FCU connection')
        while not self.state.connected:
            self.rate.sleep()

        rospy.loginfo('Got heartbeat from FCU')

        for i in range(50, 0, -1):
            self._reach_local_position(
                self.local_position.pose.position.x,
                self.local_position.pose.position.y,
                self.local_position.pose.position.z
                )

            self.rate.sleep()

        while not rospy.is_shutdown():
                self._reach_local_position(
                    self.local_position.pose.position.x,
                    self.local_position.pose.position.y,
                    self.local_position.pose.position.z
                )

                if self.state.mode != self.state.MODE_PX4_OFFBOARD:
                    self._set_mode(self.state.MODE_PX4_OFFBOARD)
                
                if self.state.armed:
                    break
                    
                self._set_arm(True)

                self.rate.sleep()

    def reach_drone_relative_position(self, x, y, z, offset, yaw = np.pi / 2):
        x_rel = x + self.local_position.pose.position.x
        y_rel = y + self.local_position.pose.position.y
        z_rel = z + self.local_position.pose.position.z

        while not self._is_at_local_position(x_rel, y_rel,z_rel, offset):

            rospy.loginfo_throttle(1, 'Attempting to reach body relative position ( x: {}, y: {}, z: {} ) | Delta ( x: {}, y: {}, z: {} )' \
                .format(
                    x, y, z,
                    abs(x_rel - self.local_position.pose.position.x),
                    abs(y_rel - self.local_position.pose.position.y), 
                    abs(z_rel - self.local_position.pose.position.z)))

            self._reach_local_position(x_rel, y_rel, z_rel, yaw)

