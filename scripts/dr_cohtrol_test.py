#!/usr/bin/env python3

import rospy
from drone.control import DroneController

if __name__ == '__main__':
    try:
        c = DroneController()   
        c.arm()
        c.reach_local_pos(5,5,5, 0.1)
        c.reach_local_pos(-5,-5,5, 0.1)
        c.reach_local_pos(0,0,-5, 0.1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
