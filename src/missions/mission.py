import roslaunch

# Missions are a roslaunch file for a set of nodes. When all of its processes exit, it completes
class Mission(object):
    def __init__(self, launcher, package, node, name):
        self.launcher = launcher
        self.name = name

        self.node = roslaunch.core.Node(package, node) 
        self.process = None

    def start(self):
        self.process = self.launcher.launch(self.node)

    def stop(self):
        self.process.stop()
