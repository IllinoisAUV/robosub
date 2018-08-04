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
