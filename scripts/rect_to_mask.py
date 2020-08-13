#!/usr/bin/env python
# -*- coding: utf-8 -*

import sys

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import RectArray


class RectToMask():
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("~output", Image, queue_size=10)
        self.get_height_width()
        self.subscribe()

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~rect', RectArray, self.callback)

    def get_height_width(self):
        ci = rospy.wait_for_message('~camera_info', CameraInfo)
        self.height = ci.height
        self.width = ci.width

    def callback(self, msg):
        mask = np.zeros((self.height, self.width), dtype=np.uint8)
        for rect in msg.rects:
            mask[rect.y:rect.y + rect.height,
                 rect.x:rect.x + rect.width] = 255
        msg_out = self.bridge.cv2_to_imgmsg(mask, "mono8")
        msg_out.header = msg.header
        self.pub.publish(msg_out)


def main(args):
    rospy.init_node("rect_to_mask", anonymous=False)
    r2m = RectToMask()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
