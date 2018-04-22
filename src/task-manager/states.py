import rospy
import smach
import smach_ros
import roslib
import sensor_checker


class Start_and_Init_Sub(smach.state):
    def __init__(self):
        smach.state.__init__(self, outcomes={'init_complete', 'init_failed', 'kill_switch'})
        self.all_checks = False
        # init the varibles needed for init testing

    def execute(self, userdata):
        # check sensors{imu, depth sensor, connection}
        sensor_check = sensor_checker.sensor_checker()

        sensor_check.listener()

        if(sensor_checker.Pressure_sensor_check and sensor_checker.Imu_check):
            self.all_checks = True
        
        if not self.all_checks:
            # transition to kill state
            return 'init_failed'

        # change depth
        # @TODO we will get DEPTH parameter from launch file
        # @TODO execute this using action_lib
        depth = 5.0
        if 4.5 <= depth <= 5.5:
