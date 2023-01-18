#!/usr/bin/env python3

from positioning.aruco import ArucoEstimator

import rospy

if __name__ == '__main__':
    try:
        ArucoEstimator().start()
    except rospy.ROSInterruptException:
        pass