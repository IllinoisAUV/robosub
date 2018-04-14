# RC Override with Mavros

We use rc overrides to control ArduSub through mavros. This means we emulate a 
joystick, and specify channels as pwm signals. Knowing what the channels do is
an annoying problem that isn't well specified by ArduPilot. The only ways to 
figure out how it works is by reading the code or by experimenting. This 
document explains what code to read to figure out what the channels mean. The 
meaning of the channels changes based on what mode the sub is in.

Each channel controls a specific degree of freedom of the sub. You can find out
what a channel controls in `ardupilot/ArduSub/radio.cpp`. Here's a basic snippet
from the current version.

```
// ardupilot/ArduSub/radio.cpp
    channel_pitch    = RC_Channels::rc_channel(0);
    channel_roll     = RC_Channels::rc_channel(1);
    channel_throttle = RC_Channels::rc_channel(2);
    channel_yaw      = RC_Channels::rc_channel(3);
    channel_forward  = RC_Channels::rc_channel(4);
    channel_lateral  = RC_Channels::rc_channel(5);
```

In each control file, you'll see how the sub turns those channels into motor outputs.

You'll see things like `float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());`. This indicates that the yaw rate is being set based on the yaw channel. Other things will 

You'll also see `get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());`. This indicates that the roll and pitch channels will directly set the value of the roll and pitch angles.
    

## Alt Hold Mode
Pitch, Roll channels directly control pitch and roll angles. 

Yaw controls the yaw rate.

Throttle, lateral and forward control the thruster commands in the given 
direction. That results in a constant thrust output in the given direction. The
sub will accelerate in that direction until friction and thrust are equal.

## Manual Mode
All channels are directly mapped to thrust around the axis.

## Stabilize Mode
Pitch, Roll channels directly control pitch and roll angles. 

Yaw controls the yaw rate.

Throttle, lateral and forward control the thruster commands in the given 
direction. That results in a constant thrust output in the given direction. The
sub will accelerate in that direction until friction and thrust are equal.
