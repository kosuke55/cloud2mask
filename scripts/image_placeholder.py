#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class MaskRepublisher():
    def __init__(self):
        self.brdge = CvBridge()
        self.pub = rospy.Publisher("~output", Image, queue_size=10)
        self.mask = None
        self.subscribe()
        rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            # '/point_indices_to_mask_image_depth/output', Image, self.callback)
            '~input', Image, self.callback)

    def callback(self, mask):
        self.mask = mask

    def timer_callback(self, timer):
        if self.mask is not None:
            self.pub.publish(self.mask)


if __name__ == '__main__':
    rospy.init_node("mask_republisher", anonymous=False)
    MaskRepublisher()
    rospy.spin()
