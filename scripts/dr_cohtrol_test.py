#!/usr/bin/env python3

import rospy
from drone.control import DroneController

if __name__ == '__main__':
    try:
        c = DroneController()   
        c.start()
        c.reach_drone_relative_position(5,5,5, 0.1)
        c.reach_drone_relative_position(-5,-5,5, 0.1)
        c.reach_drone_relative_position(0,0,0, 0.1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
