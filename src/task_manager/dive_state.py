#!/usr/bin/env python

import roslib

import rospy
import smach
import smach_ros

from actionlib import *
from actionlib_msgs.msg import *

from robosub.msg import DiveAction
from robosub.msg import DiveGoal


class Dive_State:
    def __init__(self, sm, name):
        self.sm = sm
        self.state_name = name
        self.depth_achieved = None

    def execute(self, target_depth, timeout):

        with self.sm:
            dive_action_ = smach_ros.SimpleActionState('dive/dive_action',
                            DiveAction,
                            goal = DiveGoal(depth=target_depth),
                            preempt_timeout = timeout,
                            result_cb = self.result_cb )

            smach.StateMachine.add(self.state_name, dive_action_,
                               transitions={'succeeded':'Dive_target_achieved',
                               'aborted':'Dive_action_aborted',
                               'preempted':'Dive_prevented' })

    def result_cb(self, userdata, status, res):
        if status == GoalStatus.SUCCEEDED:
            self.depth_achieved = res.depth
