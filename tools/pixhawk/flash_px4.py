#!/usr/bin/python2 -u
# From https://github.com/bluerobotics/companion/

import os
from urllib2 import urlopen
import time
import sys
import signal
from optparse import OptionParser


DIR = os.path.dirname(os.path.realpath(__file__))

def timeout(signum, frame):
    print('Timed out waiting for firmware on stdin!')
    exit(1)


parser = OptionParser()
parser.add_option("--url", dest="url", help="Firmware download URL (optional)")
parser.add_option("--stdin", action="store_true", dest="fromStdin",
                  default=False, help="Expect input from stdin")
parser.add_option("--file", dest="file", default=None, help="Load from file")
parser.add_option("--latest", action="store_true", dest="latest", default=False, help="Upload latest development firmware")
(options, args) = parser.parse_args()

if options.fromStdin:
    # Get firmware from stdin if possible
    print("Trying to read file from stdin...")

    signal.signal(signal.SIGALRM, timeout)
    signal.alarm(5)
    fileIn = sys.stdin.read()
    signal.alarm(0)

    if fileIn:
        file = open("/tmp/ArduSub-v2.px4","w")
        file.write(fileIn)
        file.close()
        print("Got firmware file from stdin!")
    else:
        raise Exception("Read error on stdin!")
elif options.file is not None:
    try:
        print("Attempting upload from file %s" % options.file)
        open(options.file)
    except Exception as e:
        print("Error opening file %s: %s" % (options.file, e))
        exit(1)
else:
    # Download most recent firmware
    if options.url:
        firmwareURL = options.url
        print("Downloading ArduSub firmware from %s" % firmwareURL)
    elif options.latest:
        firmwareURL = "http://firmware.ardupilot.org/Sub/latest/PX4/ArduSub-v2.px4"
        print("Downloading latest ArduSub firmware from %s" % firmwareURL)
    else:
        firmwareURL = "http://firmware.ardupilot.org/Sub/stable/PX4/ArduSub-v2.px4"
        print("Downloading stable ArduSub firmware from %s" % firmwareURL)

        try:
            firmwarefile = urlopen(firmwareURL)
            with open("/tmp/ArduSub-v2.px4", "wb") as local_file:
                local_file.write(firmwarefile.read())

            local_file.close()

        except Exception as e:
            print(e)
            print("Error downloading firmware! Do you have an internet connection? Try 'ping ardusub.com'")
            exit(1)


# Flash Pixhawk
print("Flashing Pixhawk...")
if options.file is not None:
    if(os.system("python -u %s/px_uploader.py --port /dev/ttyACM0 '%s'" % (DIR, options.file)) != 0):
        print("Error flashing pixhawk!")
        exit(1)
else:
    if(os.system("python2 -u %s/px_uploader.py --port /dev/ttyACM0 /tmp/ArduSub-v2.px4" % DIR) != 0):
        print("Error flashing pixhawk!")
        exit(1)


print("Complete!")