import rospy
import smach
import smach_ros
import roslib



class Start_and_Init_Sub(smach.state):
    def __init__(self):
        smach.state.__init__(self, outcomes={'init_complete', 'init_failed', 'kill_switch'})
        # init the varibles needed for init testing

    def execute(self, userdata):
        # check sensors{imu, depth sensor, connection}
        all_checks = True
        if !all_checks:
            # transition to kill state
            return 'init_failed'
        # change depth
        #we will get this parameter from launch file
        depth = 5.0
        if 4.5 <= depth <= 5.5:
