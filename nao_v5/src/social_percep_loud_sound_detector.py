#!/usr/bin/python
#-*- encoding: utf8 -*-

import rospy
from perception_base.perception_base import PerceptionBase
from geometry_msgs.msg import PointStamped

class LoudSoundDetector(PerceptionBase):
    def __init__(self):
        super(LoudSoundDetector, self).__init__("touch_sensor")

        rospy.Subscriber('loud_sound_detect', PointStamped, self.handle_loud_sound_detected)
        rospy.loginfo('[%s] initialze done...'%rospy.get_name())
        rospy.spin()

    def handle_loud_sound_detected(self, msg):
        write_data = self.conf_data['loud_sound_detection']['data']
        write_data['xyz'] = [msg.point.x, msg.point.y, msg.point.z]
        write_data['frame_id'] = msg.header.frame_id

        if msg.point.x >= 0:
            if msg.point.y > 0.2:
                write_data['direction'] = 'front_left'
            elif msg.point.y < -0.2:
                write_data['direction'] = 'front_right'
            else:
                write_data['direction'] = 'front'
        else:
            if msg.point.y > 0.2:
                write_data['direction'] = 'rear_left'
            elif msg.point.y < -0.2:
                write_data['direction'] = 'rear_right'
            else:
                write_data['direction'] = 'rear'

        self.save_to_memory(self.conf_data.keys()[0], data=write_data)
        self.raise_event(self.conf_data.keys()[0], 'loud_sound_detected')

if __name__ == '__main__':
    m = LoudSoundDetector()
