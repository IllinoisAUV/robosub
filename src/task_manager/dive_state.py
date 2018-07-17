#!/usr/bin/env python

import roslib

import rospy
import smach
import smach_ros

from robosub.DiveAction.msg import DiveAction
from robosub.DiveGoal.msg import DiveGoal

from actionlib import *
from actionlib_msgs.msg import *


class Dive_State(object):
    def __init__(self, sm, name, depth_achieved):
        self.sm = sm
        self.state_name = name
        self.depth_achieved = depth_achieved

    def execute(self, target_depth, timeout):

        with self.sm:
            dive_action_ = smach_ros.SimpleActionState('Dive_Action', DiveAction,
                               goal = DiveGoal(goal=target_depth),
                               preempt_timeout = rospy.Duration(timeout),
                               result_slots=['depth_achieved'])

            smach.StateMachine.add(self.state_name, dive_action_,
                               transitions={'succeeded':'succeeded',
                               'aborted':'aborted',
                               'preempted':'preempted' },
                               remapping= {'depth_achieved':'userdata_depth'})

        outcome = self.sm.execute()
        return

    def result_cb(userdata, status, result):
        if status == GoalStatus.SUCCEEDED:
            self.sm.userdata.depth_achieved = result.depth_achieved
        return
