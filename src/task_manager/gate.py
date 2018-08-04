#!/usr/bin/env python
import rospy

from mover import Mover
from task import Task
from config import Config

class Gate(Task):
    def __init__(self):
        self.mover = Mover()

    def run(self):
        self.mover.dive(Config.gate_depth_time, Config.gate_depth_speed)
        self.mover.forward(Config.gate_forward_time, Config.gate_forward_speed)

