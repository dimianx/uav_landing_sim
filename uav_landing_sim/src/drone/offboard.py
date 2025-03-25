import rospy
import math
import numpy as np

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Quaternion
from geographic_msgs.msg import GeoPoseStamped
from pygeodesy.geoids import GeoidPGM
from sensor_msgs.msg import NavSatFix
from numpy.linalg import norm

from tf.transformations import quaternion_from_euler

class OffboardController():

    def __init__(self):
        self.egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)
        
        self.state = State()
        self.local_position  = [0] * 3
        self.orientation = [0] * 4
        self.global_position = [0] * 3
        self.home_position   = [0] * 3
        self.next_vel_waypoint = [0] * 3
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
            rospy.wait_for_service('/mavros/cmd/command', 30)

        except rospy.ROSException:
            rospy.logerr('Failed to connect to services')

        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_command_srv = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

    def _setup_subscribers(self):
        rospy.loginfo('Subscribing to mavros topics')

        self.state_sub = rospy.Subscriber('/mavros/state', State, self._state_callback)
        self.local_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_position_callback)
        self.global_pose_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self._global_position_callback)
        self.traj_sub = rospy.Subscriber('/landing/setpoint_vel', TwistStamped, self._trajectory_callback)

    def _setup_publishers(self):
        self.pos_setpoint_local_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped)
        self.pos_setpoint_global_pub = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped)
        self.pos_setpoint_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped)

    def _state_callback(self, msg):
        self.state = msg

    def _local_position_callback(self, msg):
        self.local_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]

        self.orientation = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

    def _global_position_callback(self, msg):
        self.global_position = [
            msg.latitude,
            msg.longitude,
            msg.altitude
        ]

    def _trajectory_callback(self, msg):
        self.next_vel_waypoint = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ]


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

    def _is_started(self):
        return self.state.mode == self.state.MODE_PX4_OFFBOARD and self.state.armed
    
    def _set_local_goal(self, xyz, yaw = np.pi / 2):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position = Point(xyz[0], xyz[1], xyz[2])
        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = Quaternion(*quaternion)

        self.pos_setpoint_local_pub.publish(msg)

    def _set_gps_goal(self, latlon, yaw = np.pi / 2):
        msg = GeoPoseStamped()

        msg.pose.position.latitude = latlon[0]
        msg.pose.position.longitude = latlon[1]
        msg.pose.position.altitude = self.global_position[2] - self.egm96.height(latlon[0], latlon[1])

        quaternion = quaternion_from_euler(0,0, yaw)
        msg.pose.orientation = Quaternion(*quaternion)

        self.pos_setpoint_global_pub.publish(msg)


    def arm(self, timeout = 30.0):
        t = rospy.Time.now()

        rospy.loginfo('Starting drone controller')

        rospy.loginfo('Waiting for FCU connection')
        while not self.state.connected:
            self.rate.sleep()

        rospy.loginfo('Got heartbeat from FCU')

        for i in range(50, 0, -1):
            self._set_local_goal(self.local_position)
            self.rate.sleep()

        while not rospy.is_shutdown():
            self._set_local_goal(self.local_position)

            if self.state.mode != self.state.MODE_PX4_OFFBOARD:
                self._set_mode(self.state.MODE_PX4_OFFBOARD)
                
            if self.state.armed:
                break
                    
            self._set_arm(True)

            if rospy.Time.now() - t > rospy.Duration(timeout):
                raise Exception('Unable to arm')

            self.rate.sleep()
                

    def force_disarm(self):
        if not self._is_started:
            raise Exception('Not armed')
        
        try:
            self.set_command_srv(
                command = 400,
                confirmation = 0,
                param1 = 0,
                param2 = 21196,
                param3 = 0,
                param4 = 0,
                param5 = 0,
                param6 = 0,
                param7 = 0
            )
            
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def reach_relative_goal(self, xyz, offset, yaw = None):
        if not self._is_started:
            raise Exception('Not armed')

        xyz_rel = [x + y for x, y in zip(xyz, self.local_position)]
        lpos = self.local_position

        while norm([x - y for x, y in zip(xyz_rel, self.local_position)]) > offset:
            n = [x - y for x, y in zip(xyz_rel, lpos)] / np.linalg.norm([x - y for x, y in zip(xyz_rel, lpos)])
            lpos += 0.1 * n

            if yaw is None:
                dx = xyz_rel[0] - self.local_position[0]
                dy = xyz_rel[1] - self.local_position[1]
                n_yaw = math.atan2(dy, dx)
            else:
                n_yaw = yaw

            self._set_local_goal(xyz_rel, n_yaw)
            self.rate.sleep()
        
        rospy.loginfo('Target reached')

    def reach_gps_goal(self, latlon, offset, yaw = np.pi / 2):
        if not self._is_started:
            raise Exception('Not armed')
        
        while norm(np.array([latlon]) - np.array(self.global_position[:-1])) > offset:
            self._set_gps_goal(latlon, yaw)
            self.rate.sleep()

        rospy.loginfo('Target reached')
    
    def set_local_goal(self, xyz, yaw = None):
        if yaw is None:
            dx = xyz[0] - self.local_position[0]
            dy = xyz[1] - self.local_position[1]
            n_yaw = math.atan2(dy, dx)
        else:
            n_yaw = yaw
    
        self._set_local_goal(xyz, n_yaw)
        self.rate.sleep()

    def get_local_position(self):
        return self.local_position

    def get_next_vel_setpoint(self):
        return self.next_vel_waypoint
    
    def set_velocity_goal(self, vxvyvz):
        if not self._is_started:
            raise Exception('Not armed')
        
        msg = TwistStamped()
        msg.twist.linear.x = vxvyvz[0]
        msg.twist.linear.y = vxvyvz[1]
        msg.twist.linear.z = vxvyvz[2]

        self.pos_setpoint_vel_pub.publish(msg)
        self.rate.sleep()

