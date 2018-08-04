#!/usr/bin/env python
import rospy

from mover import Mover
from task import Task

class Gate(Task):
    depth_speed = -0.4
    depth_time = 7
    forward_speed = 0.2
    forward_time = 27.0

    def __init__(self):
        self.mover = Mover()

    def run(self):
        self.mover.dive(self.depth_time, self.depth_speed)
        self.mover.forward(self.forward_time, self.forward_speed)

