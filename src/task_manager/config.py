#!/usr/bin/env python


class Config(object):
    visual_servo_kp_alt = 0.003
    visual_servo_kp_yaw = 0.003
    visual_servo_forward_speed = 0.2
    mover_topic = "/cmd_vel"
    arming_topic = "/arming"
    darknet_topic = "/darknet_ros/bounding_boxes"
    zed_camera_dims = (1280, 720)

    dice_yaw_speed = 0.2
    dice_yaw_time = 0.7

    gate_depth_speed = -0.4
    gate_depth_time = 7
    gate_forward_speed = 0.2
    gate_forward_time = 27.0
