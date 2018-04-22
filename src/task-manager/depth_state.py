#!/usr/bin/env python

import roslib

import rospy
import smach
import smach_ros

from robosub.DiveAction.msg import DiveAction
from robosub.DiveGoal.msg import DiveGoal

from actionlib import *
from actionlib_msgs.msg import *

class Sensor_Checks(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Checks_passed'])

    def execute(self, userdata):
        # do all the checks
        rospy.loginfo("Sensor_Checks")
        return 'Checks_passed'


class Dive_State:
    def __init__(self, sm, name, depth_achieved):
        self.sm = sm
        self.state_name = name
        self.depth_achieved = depth_achieved
    class

    def execute(self, target_depth, timeout):

        with self.sm:
            dive_action_ = smach_ros.SimpleActionState('Dive_Action', DiveAction,
                               goal = DiveGoal(goal=target_depth),  preempt_timeout = rospy.Duration(timeout),
                               result_slots=['depth_achieved'])

            smach.StateMachine.add(self.state_name, dive_action_,
                               transitions={'succeeded':'succeeded', 'aborted':'aborted', 'preempted':'preempted' },
                               remapping= {'depth_achieved':'userdata_depth'})

    def result_cb(userdata, status, result):
        if status == GoalStatus.SUCCEEDED:
            self.sm.userdata.depth_achieved = result.depth_achieved
        return



def main():
    rospy.init_node('Robosub_StateMachine')

    sm_top = smach.StateMachine(outcomes=['Mission_Completed'])
    with sm_top:

         smach.StateMachine.add('Sensor_Checks', Sensor_Checks(),
                               transitions={'Checks_passed':'Dive'})

        dive_sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

        dive_state_ = Dive_State(dive_sm, 'Dive')
        # target depth in mts and preempted timeout in sec
        # final outcome to return
        dive_state_.execute(5.0, 60.0)

        depth_achieved = dive_sm.userdata.depth_achieved

        smach.StateMachine.add('dive_sm', dive_sm,
                    transitions={'succeeded':'Mission_Completed', 'aborted': 'Some_state', 'preempted':'Some_state'} )

    outcome = sm_top.execute()

    rospy.signal_shutdown('All_done.')


if __name__ == '__main__':
    main()
