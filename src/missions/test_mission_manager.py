#!/usr/bin/env python2
'''
Test cases for the mission manager
'''
import subprocess
import os
import signal
import time
import unittest
import rospkg

from missions import MissionManager

class TestMissionManager(unittest.TestCase):
    '''
    Tests for mission manager
    '''
    def setUp(self):
        '''
        Set up all of the reused variables for the tests
        '''
        self.rospack = rospkg.RosPack()
        self.basedir = self.rospack.get_path('robosub')

    # Test to make sure no errors occur
    def test_start_single(self):
        '''
        Test running a single mission
        '''
        missions = [os.path.join(self.basedir, 'launch/test/test_node.launch')]
        manager = MissionManager()
        manager.run(missions)

    def test_start_multiple(self):
        '''
        Test running multiple missions
        '''
        missions = [os.path.join(self.basedir, 'launch/test/test_node.launch')]*5
        manager = MissionManager()
        manager.run(missions)

if __name__ == '__main__':
    # Prestart roscore
    ROSCORE = subprocess.Popen('roscore', shell=True, preexec_fn=os.setsid)
    time.sleep(1)

    import rosunit
    rosunit.unitrun('test_missions', 'test_mission_manager', TestMissionManager)

    # See
    # https://stackoverflow.com/questions/4789837/how-to-terminate-a-python-subprocess-launched-with-shell-true/4791612
    os.killpg(os.getpgid(ROSCORE.pid), signal.SIGINT)
