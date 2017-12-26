#!/usr/bin/env python2
import subprocess
import os
import signal
import time
from cStringIO import StringIO
import re

import unittest
from missions import MissionManager
import rospkg
import sys

class TestMissionManager(unittest.TestCase):
    def setUp(self):
        self.rospack = rospkg.RosPack()
        self.basedir = self.rospack.get_path('robosub')


    def tearDown(self):
        pass

    # Test to make sure no errors occur
    def test_start_single(self):
        missions = [os.path.join(self.basedir, 'launch/test/test_node.launch')]
        manager = MissionManager()
        manager.run(missions)

    def test_start_multiple(self):
        missions = [os.path.join(self.basedir, 'launch/test/test_node.launch')]*5
        manager = MissionManager()
        manager.run(missions)

if __name__ == '__main__':
    # Prestart roscore 
    roscore = subprocess.Popen('roscore', shell=True, preexec_fn=os.setsid)
    time.sleep(1)

    import rosunit
    rosunit.unitrun('test_missions', 'test_mission_manager', TestMissionManager)

    # See https://stackoverflow.com/questions/4789837/how-to-terminate-a-python-subprocess-launched-with-shell-true/4791612
    os.killpg(os.getpgid(roscore.pid), signal.SIGINT)
