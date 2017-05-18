#!/usr/bin/env python
#-*- encoding: utf8 -*-

import math
import threading
import rospy
import tf
import numpy as np

from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from mhri_msgs.msg import GazeCommand
from geometry_msgs.msg import PointStamped


class HeadJointLimits:
    YAW_LIMIT_MIN = (-119.5 * math.pi / 180.0)
    YAW_LIMIT_MAX = (119.5 * math.pi / 180.0)
    PITCH_LIMIT_MIN = (-38.5 * math.pi / 180.0)
    PITCH_LIMIT_MAX = (29.5 * math.pi / 180.0)

class GazeRenderNode:
    def __init__(self):
        rospy.init_node('nao_render_gaze', anonymous=False)

        self.lock = threading.RLock()
        self.listener = tf.TransformListener()

        rospy.Subscriber('gaze_command', GazeCommand, self.handle_gaze_point)
        self.pub_gaze_cmd = rospy.Publisher(
            'joint_angles', JointAnglesWithSpeed, queue_size=1)

        with self.lock:
            self.target = GazeCommand()
            self.target.target_point.header.frame_id = "base_footprint"
            self.target.target_point.point.x = 2.0
            self.target.target_point.point.y = 0.0
            self.target.target_point.point.z = 0.6
            self.target.max_speed = 0.1

        rospy.Timer(rospy.Duration(0.1), self.handle_gaze_controller)
        rospy.loginfo('[%s] initialzed...'%rospy.get_name())
        rospy.spin()

    def handle_gaze_controller(self, event):
        try:
            with self.lock:
                point_transformed = self.listener.transformPoint('gaze', self.target.target_point)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("[%s] Can't tranform from gaze to target."%rospy.get_name())
            return

        pan_angle = math.atan2(point_transformed.point.y, point_transformed.point.x)
        tilt_angle = math.atan2(point_transformed.point.z, point_transformed.point.x)

        delta_pan_angle = pan_angle
        delta_tilt_angle = -tilt_angle

        cmd_pan_angle = 0.0
        cmd_tilt_angle = 0.0

        if abs(delta_pan_angle) < (2.0 * math.pi / 180.0):
            cmd_pan_angle = 0.0
        else:
            cmd_pan_angle = 0.3 * delta_pan_angle

        if abs(delta_tilt_angle) < (2.0 * math.pi / 180.0):
            cmd_tilt_angle = 0.0
        else:
            cmd_tilt_angle = 0.3 * delta_tilt_angle


        cmd_msg = JointAnglesWithSpeed()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.joint_names.append('HeadYaw')
        cmd_msg.joint_angles.append(cmd_pan_angle)
        cmd_msg.joint_names.append('HeadPitch')
        cmd_msg.joint_angles.append(cmd_tilt_angle)
        cmd_msg.speed = self.target.max_speed
        cmd_msg.relative = 1

        self.pub_gaze_cmd.publish(cmd_msg)

    def handle_gaze_point(self, msg):
        with self.lock:
            self.target = msg


if __name__ == '__main__':
    m = GazeRenderNode()
    rospy.spin()
