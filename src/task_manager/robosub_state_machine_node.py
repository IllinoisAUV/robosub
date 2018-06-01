#!/usr/bin/env python
import roslib

import rospy
import smach
import smach_ros

# states
from sensor_checker_state import Sensor_Checks
from dive_state import Dive_State

from actionlib import *
from actionlib_msgs.msg import *

def main():

    rospy.init_node('Robosub_StateMachine')

    '''
    Robosub state machine.
    Final Outputs:
        Mission_Completed : All tasks were completed succesfully
        Mission_Failed : All tasks were not completed succesfully
        System_Failure : The tasks were not completed succesfully due to technical failure
    '''
    sm_top = smach.StateMachine(outcomes=['Mission_Completed', 'Mission_Failed', 'System_Failure'])

    with sm_top:

        #############################################################################################

        '''
        ### Sensor checks state ###

        Sensor_Checks state init with Time out time.
        Basic smach state, not an action state.
        Final Outputs:
            Checks_passed: if all the sensor are working properly.
            Checks_failed: if all the sensors are not working properly.
        '''
        smach.StateMachine.add('Sensor_Checks', Sensor_Checks(10),
                               transitions={'Checks_passed':'dive_sm', 'Checks_failed':'Mission_Failed'})


       #############################################################################################
        '''
        ### Dive State Machine ###
        State to change the depth to a desired amount.
        Action state.
        Final Outputs:
            Dive_target_achieved: Achieved the desired depth.
            Dive_target_aborted: The action was aborted due to some kind of failure
            Dive_prevented: Did not achieve the desired depth.
        '''
        dive_sm = smach.StateMachine(outcomes=['Dive_target_achieved','Dive_action_aborted','Dive_prevented'])

        dive_state_ = Dive_State(dive_sm, 'Dive')

        # target depth and preempted timeout in sec
        # final outcome to return
        # @TODO check if timeout working correctly
        dive_state_.execute(-5.0, 1.0)

        smach.StateMachine.add('dive_sm', dive_sm,
                               transitions={'Dive_target_achieved':'Mission_Completed', 'Dive_action_aborted': 'Mission_Failed', 'Dive_prevented':'Mission_Failed'} )

        #############################################################################################

    outcome = sm_top.execute()

    print(dive_state_.depth_achieved)

    rospy.signal_shutdown('All_done.')

if __name__ == '__main__':
    main()
