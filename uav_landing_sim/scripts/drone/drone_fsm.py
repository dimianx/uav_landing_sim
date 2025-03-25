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
import csv
import os
import traceback
import numpy as np

from smach import State, StateMachine
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Bool
from drone.offboard import OffboardController

class Start(State):
    def __init__(self):
        State.__init__(self, outcomes = ['mission_acquired', 'failed'])
        self.count = 0

    def execute(self, _):
        try:
            rospy.loginfo('[FSM] Executing state START')
            self.count += 1

            attempts_count = rospy.get_param('~attempts_count')
            if self.count > attempts_count:
                rospy.delete_param('~target_waypoint')
                rospy.delete_param('~takeof_altitude')

            while not (rospy.has_param('~target_waypoint') or rospy.has_param('~takeoff_altitude')):
                rospy.loginfo_once('[FSM] Waiting for target waypoint. Load target_waypoint and takeoff_altitude params.')

            rospy.loginfo('[FSM] Received target_waypoint and takeoff_height')
            return 'mission_acquired'
        except Exception as e:
            rospy.logerr(e)
            return 'failed'
        
class Takeoff(State):
    def __init__(self, drc):
        State.__init__(self, outcomes = ['takeoff_altitude_reached', 'failed'])
        self.drc = drc

    def execute(self, _):
        try:
            rospy.loginfo('[FSM] Executing state TAKEOFF')

            takeoff_altitude = rospy.get_param('~takeoff_altitude')

            self.drc.arm(timeout = 35.0)
            self.drc.reach_relative_goal([0, 0, takeoff_altitude], yaw = np.pi / 2, offset=0.15)
            
            return 'takeoff_altitude_reached'

        except Exception as e:
            traceback.print_exc()
            rospy.logerr(e)
            return 'failed'

class Mission(State):
    def __init__(self, drc, ena_pub):
        State.__init__(self, outcomes = ['mission_completed', 'failed'])
        self.drc = drc
        self.ena_pub = ena_pub
        
    def execute(self, _):
        try:
            rospy.loginfo('Executing state MISSION')

            target_waypoint = rospy.get_param('~target_waypoint')
            self.drc.reach_gps_goal(target_waypoint, yaw = np.pi / 2, offset = 1e-6)
                        
            msg_ena = Bool()
            msg_ena.data = True
            self.ena_pub.publish(msg_ena)
            rospy.loginfo('Landing controller enabled')

            lpos = self.drc.get_local_position()

            while self.drc.get_next_vel_setpoint()[0] == 0.0 \
                and self.drc.get_next_vel_setpoint()[1] == 0.0:
                self.drc.set_local_goal(lpos, yaw = np.pi / 2)
            
            return 'mission_completed'

        except Exception as e:
            rospy.logerr(e)
            traceback.print_exc()
            return 'failed'

class Landing(State):
    def __init__(self, drc, ena_pub):
        State.__init__(self, outcomes = ['landing_completed', 'target_missed', 'failed'])
        self.drc = drc
        self.ena_pub = ena_pub

    def execute(self, _):
        try:
            rospy.loginfo('[FSM] Executing state LANDING')

            t = rospy.Time.now()
            while not rospy.is_shutdown():
                goal = self.drc.get_next_vel_setpoint()
                
                if not np.isnan(goal[2]):
                    if np.isinf(goal[2]):
                        rospy.logwarn('[FSM] Target missed. Returning to state MISSION')
                        return 'target_missed'
                    
                    self.drc.set_velocity_goal(goal)
                else:
                    break

            self.drc.force_disarm()
            
            rospy.loginfo('[FSM] Landing completed')
            time = rospy.Time.now() - t
            rospy.loginfo('[FSM] Took {} seconds to land.'.format(time.to_sec()))

            try:
                rospy.wait_for_service('/gazebo/get_model_state', 5)
            except rospy.ROSException:
                rospy.logerr('[FSM] Failed to connect to ModelState service')

            model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            error_resp = model_state('iris_bottom_fpv', 'husky')
            error_pos = np.array([error_resp.pose.position.x, error_resp.pose.position.y])

            error = np.linalg.norm(error_pos)
            rospy.loginfo('[FSM] Landing error: {}'.format(error))    

            msg_disa = Bool()
            msg_disa.data = False
            self.ena_pub.publish(msg_disa)

            file = rospy.get_param('~output')
            exists = os.path.exists(file)
            with open(file, 'a', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)

                if not exists:
                    writer.writerow(['error', 'time'])

                writer.writerow(['{}'.format(error), '{}'.format(time.to_sec())])


            return 'landing_completed'
        except Exception as e:
            rospy.logerr(e)
            return 'failed'       

def start():
    rospy.init_node('drone_fsm')
    rospy.loginfo('Initialized drone_fsm node')

    sm = StateMachine(outcomes = ['FAULT'])
    d = OffboardController()
    ena_pub = rospy.Publisher('/landing/enabled', Bool, queue_size = 10)


    with sm:
        StateMachine.add('START', Start(),
                          transitions = {'mission_acquired' : 'TAKEOFF',
                                         'failed' : 'FAULT'})
        StateMachine.add('TAKEOFF', Takeoff(d),
                          transitions = {'takeoff_altitude_reached' : 'MISSION',
                                         'failed' : 'FAULT'})        
        StateMachine.add('MISSION', Mission(d, ena_pub),
                          transitions = {'mission_completed' : 'LANDING',
                                         'failed' : 'FAULT'})                
        StateMachine.add('LANDING', Landing(d, ena_pub),
                          transitions = {'landing_completed' : 'START',
                                         'target_missed' : 'MISSION',
                                         'failed' : 'FAULT'})

    outcome = sm.execute()

    rospy.spin()

if __name__ == '__main__':
    start()