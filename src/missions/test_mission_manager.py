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

    def count_runs(self, missions):
        manager = MissionManager()

        # Capture stdout
        sys.stdout = stdout = StringIO()
        sys.stderr = stderr = StringIO()
        manager.run(missions)
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__


        print(stdout.getvalue())
        return len(re.findall(r'Test node run', stdout.getvalue()))

    def test_start_single(self):
        missions = [os.path.join(self.basedir, 'launch/test/test_node.launch')]
        count = self.count_runs(missions)
        self.assertEqual(len(missions), count)



    def test_start_multiple(self):
        missions = [os.path.join(self.basedir, 'launch/test/test_node.launch')]*5
        count = self.count_runs(missions)
        self.assertEqual(len(missions), count)

if __name__ == '__main__':
    # Prestart roscore 
    roscore = subprocess.Popen('roscore', shell=True, preexec_fn=os.setsid)
    time.sleep(1)

    import rosunit
    rosunit.unitrun('test_missions', 'test_mission_manager', TestMissionManager)

    # See https://stackoverflow.com/questions/4789837/how-to-terminate-a-python-subprocess-launched-with-shell-true/4791612
    os.killpg(os.getpgid(roscore.pid), signal.SIGINT)
