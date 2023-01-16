#!/usr/bin/env python3

from positioning.uwb_lqr_vanc import LQRVANCEstimator

import rospy

if __name__ == '__main__':
    try:
        LQRVANCEstimator().start()
    except rospy.ROSInterruptException:
        pass