# Common issues with Ardupilot and how to fix them

### Connection lost to jetson leaves ardupilot running and armed
Created heartbeat.sh script to disarm when connection is lost

### Joystick complains about losing connection
Use /dev/input/js0 instead of /dev/f310controller in bluerov_apps/launch/teleop_f310.launch

### /mavros/state not publishing
Restart pixhawk, mess with uart connection

### Joystick not connecting to bluerov apps
Make sure joystick is in X, with mode light turned off.  Also, Don't use bluerov_apps teleop_f310

### No pixhawk connection to jetson
Generally, check TX/RX wiring on jetson side. Or wiggle the pixhawk side wiring. 

You can determine if this is the case by editing $ROS_ROOT/config/rosconsole.config and setting
`log4j.logger.ros=INFO` to `log4j.logger.ros=DEBUG`. Next time you run mavros, you should see a message every time a serial packet is received. If you don't see messages mentioning serial, the serial connection is not working properly.

### Camera not connecting to Jetson
Check camera usb cable on a laptop. If it doesn't work, the cable is bad

### Can't control motors through rcoverride AFTER running teleop_f310.launch, other modes still work
Restart mavros and DONT USE TELEOP_F310


