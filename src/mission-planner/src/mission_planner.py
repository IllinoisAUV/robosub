#!/usr/bin/env python

import rospy
import os
import json
import roslaunch

#Just some test missions to test with.
missions = {
    "default": None,
    "mission1": [("test_package", "test_package_exe")]
}

processes = {
    "mission1":[]
}

def start_mission (mission_name, launch):
    if (mission_name in missions):
        print mission_name
        mission_packages = missions[mission_name]
        if (mission_packages is not None):
            print mission_packages
            for (package in mission_packages):
                node = roslaunch.core.Node(package[0], package[1])
                process = launch.launch(node)
                processes[mission_name].append(process)
                print process.is_alive()
            return "Starting mission: " + mission_name
        else:
            return "Running default do nothing state."
    else:
        return "Not a valid mission."

def stop_mission (mission_name):
    if (mission_name in processes):
        mission_processes = processes[mission_name]
        for (process in mission_processes):
            process.stop()
        return "Stopped all missions"
    else:
        return "Not a valid mission."


if __name__ == '__main__':
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    with open('mission-config.json') as mission_config:
        print type(mission_config)
        config = json.load(mission_config)
        start_mission(config['mission']['current_mission'], launch)

    #need to add rospy subscriber that listens to kill commands
    #rospy subscriber for kill commands may not be the best
