#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
from perception_base.perception_base import PerceptionBase
from naoqi_bridge_msgs.msg import Bumper, HeadTouch, HandTouch
from std_msgs.msg import Bool

class TouchSensor(PerceptionBase):
    def __init__(self):
        super(TouchSensor, self).__init__("touch_sensor")

        rospy.Subscriber('foot_contact', Bool, self.handle_robot_lifted)
        rospy.Subscriber('bumper', Bumper, self.handle_bumper_event)
        rospy.Subscriber('tactile_touch', HeadTouch, self.handle_head_touch_event)
        rospy.Subscriber('hand_touch', HandTouch, self.handle_hand_touch_event)

        rospy.loginfo('[%s] initialze done...'%rospy.get_name())
        rospy.spin()

    def handle_robot_lifted(self, msg):
        if not msg.data:
            write_data = self.conf_data['touch_sensor']['data']
            write_data['touched_part'] = 'robot_lifted'

            self.save_to_memory(self.conf_data.keys()[0], data=write_data)
            self.raise_event(self.conf_data.keys()[0], 'robot_lifted')

    def handle_bumper_event(self, msg):
        if msg.state != 1:
            return

        event_name = '_bumper'
        if msg.bumper == 0:
            event_name = 'right' + event_name
        else:
            event_name = 'left' + event_name

        write_data = self.conf_data['touch_sensor']['data']
        write_data['touched_part'] = event_name

        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
        self.raise_event(self.conf_data.keys()[0], event_name + '_touched')

    def handle_head_touch_event(self, msg):
        if msg.state != 1:
            return

        event_name = '_head'
        if msg.button == 1:
            event_name = 'front' + event_name
        elif msg.button == 2:
            event_name = 'middle' + event_name
        elif msg.button == 3:
            event_name = 'rear' + event_name
        else:
            return

        write_data = self.conf_data['touch_sensor']['data']
        write_data['touched_part'] = event_name

        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
        self.raise_event(self.conf_data.keys()[0], event_name + '_touched')

    def handle_hand_touch_event(self, msg):
        if msg.state != 1:
            return

        event_name = '_hand'
        if msg.hand == 3 or msg.hand == 4 or msg.hand == 5:
            event_name = 'left' + event_name
        elif msg.hand == 0 or msg.hand == 1 or msg.hand == 2:
            event_name = 'right' + event_name

        write_data = self.conf_data['touch_sensor']['data']
        write_data['touched_part'] = event_name

        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
        self.raise_event(self.conf_data.keys()[0], event_name + '_touched')


if __name__ == '__main__':
    m = TouchSensor()
