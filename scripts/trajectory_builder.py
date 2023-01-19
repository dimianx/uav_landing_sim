#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped

class TrajectoryBuilder():

    def __init__(self):
        rospy.init_node('trajectory_builder', anonymous=True)
        rospy.loginfo('Initialized trajectory_builder node')

        self._read_config()
        self._setup_subscribers()
        self._setup_publishers()

        self.buffer = [None] * 25
        self.counter = 0
        
        self.is_landed = False

    def _read_config(self):
        self.pose_topic = rospy.get_param('~pose_topic', '')
        self.xy_tolerance = rospy.get_param('~xy_tolerance', 0.1)
        self.z_tolerance = rospy.get_param('~z_tolerance', 0.1)
        self.landing_step = rospy.get_param('~landing_step', 0.1)

    def _setup_subscribers(self):
        rospy.loginfo('Subscribing to {}'.format(self.pose_topic))
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self._positioning_callback)

    def _setup_publishers(self):
        rospy.loginfo('Will publish to /trajectory/setpoint')
        self.traj_pub = rospy.Publisher('/trajectory/setpoint', PointStamped, queue_size = 10)

    def _positioning_callback(self, msg : PoseStamped):
        self.buffer[self.counter] = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        self.counter += 1
        if self.counter >= len(self.buffer):
            self.counter = 0

        mean = self._compute_mean()
        point = self._compute_next_waypoint(mean[0], mean[1], mean[2])
        point.header.stamp = rospy.Time.now()

        rospy.loginfo(point)
        self.traj_pub.publish(point)
        
    def _compute_next_waypoint(self, x, y, z):
        p = PointStamped()

        if self.is_landed:
            p.point.x = 0
            p.point.y = 0
            p.point.z = 0
            return p

        if x > self.xy_tolerance or y > self.xy_tolerance:
            p.point.x = x
            p.point.y = y
            p.point.z = 0        
        else:
            p.point.x = x
            p.point.y = y
            if z > self.z_tolerance:
                p.point.z = self.landing_step
            else:
                # Final approach
                p.point.z = z
                self.is_landed = True
        

        return p

    def _compute_mean(self):
        num = 0
        mean_pose = [0] * 3
        for i in self.buffer:
            if i is not None:
                num += 1
                for j in range(len(mean_pose)):
                    mean_pose[j] += i[j]
        if num != 0:
            for i in range(len(mean_pose)):
                mean_pose[i] = mean_pose[i] / num

        return mean_pose

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        TrajectoryBuilder().start()
    except rospy.ROSInterruptException:
        pass