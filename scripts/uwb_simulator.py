#!/usr/bin/env python3

from sensor_sim.uwb.simulation import UWBSimulator

import rospy

if __name__ == '__main__':
    try:
        UWBSimulator().publish_ranges()
    except rospy.ROSInterruptException:
        pass