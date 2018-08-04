#!/usr/bin/env python

import roslib

import rospy
import smach
import smach_ros

from actionlib import *
from actionlib_msgs.msg import *

from robosub.msg import HitDieAction
from robosub.msg import HitDieGoal


class Dice_State:
    def __init__(self, sm, name):
        self.sm = sm
        self.state_name = name
        self.dice_hit = None

    def execute(self, target_dice, timeout):

        with self.sm:
            dice_action_ = smach_ros.SimpleActionState('hitdie/dice_action',
                            HitDieAction,
                            goal = HitDieGoal(depth=target_dice),
                            preempt_timeout = timeout,
                            result_cb = self.result_cb )

            smach.StateMachine.add(self.state_name, dice_action_,
                               transitions={'succeeded':'Dice_hit',
                               'aborted':'Dice_action_aborted',
                               'preempted':'Dice_action_prevented' })

    def result_cb(self, userdata, status, res):
        if status == GoalStatus.SUCCEEDED:
            self.dice_hit = res.success
