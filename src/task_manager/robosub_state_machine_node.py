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

    sm_top = smach.StateMachine(outcomes=['Mission_Completed', 'Mission_Failed'])

    with sm_top:

        # sensor check state with 10 sec timeout
        # basic smach state, no action state server
        smach.StateMachine.add('Sensor_Checks', Sensor_Checks(10),
                               transitions={'Checks_passed':'Dive', 'Checks_failed':'Mission_Failed'})

        dive_sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

        dive_state_ = Dive_State(dive_sm, 'Dive')

        # target depth in mts and preempted timeout in sec
        # final outcome to return
        dive_state_.execute(5.0, 60.0)

        depth_achieved = dive_sm.userdata.depth_achieved

        smach.StateMachine.add('dive_sm', dive_sm,
                    transitions={'succeeded':'Mission_Completed', 'aborted': 'Mission_Failed', 'preempted':'Mission_Failed'} )

    outcome = sm_top.execute()

    rospy.signal_shutdown('All_done.')

if __name__ == '__main__':
    main()
