import roslaunch
from threading import Thread, Lock, Condition
import rospy


class MissionManager(object):
    """MissionManager controls serializing missions and running them
    Note: roscore must be running separately for missions to properly exit
    Args: None
    Attributes: None
    """
    def __init__(self):
        rospy.init_node('mission_manager')

    def run(self, missions):
        # mission is a path to a launch file
        for mission in missions:
            # From http://wiki.ros.org/roslaunch/API%20Usage
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [mission], force_screen=True)

            print("Mission %s starting" % mission)
            launch.start()

            # Spin until all the launched processes exit
            launch.spin()

            print("Mission %s done" % mission)
