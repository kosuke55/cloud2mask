#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ExpandMask():
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("~output", Image, queue_size=10)
        self.padding = rospy.get_param('~padding', 20)
        self.subscribe()

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input', Image, self.callback)

    def callback(self, msg):
        mask = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        height, width = mask.shape
        foreground = np.where(mask == 255)

        if len(foreground[0]) > 0:
            ymin = np.min(foreground[0]) - self.padding
            ymax = np.max(foreground[0]) + self.padding
            xmin = np.min(foreground[1]) - self.padding
            xmax = np.max(foreground[1]) + self.padding

            ymin = max(0, ymin)
            ymax = min(height, ymax)
            xmin = max(0, xmin)
            xmax = min(height, xmax)

            mask[ymin:ymax, xmin:xmax] = 255

        msg_out = self.bridge.cv2_to_imgmsg(mask, 'mono8')
        msg_out.header = msg.header
        self.pub.publish(msg_out)


if __name__ == '__main__':
    rospy.init_node("expand_mask", anonymous=False)
    ExpandMask()
    rospy.spin()
