#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# from std_srvs.srv import SetBool, SetBoolResponse
from std_srvs.srv import Trigger, TriggerResponse


class ImagePlaceHolder():
    def __init__(self):
        self.brdge = CvBridge()
        self.pub = rospy.Publisher("~output", Image, queue_size=10)
        self.img = None
        self.callback_update = rospy.get_param(
            '~callback_update', True)
        self.subscribe()
        self.service()
        rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def service(self):
        self.service = rospy.Service(
            'set_image', Trigger, self.set_image)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input', Image, self.callback)

    def callback(self, img):
        if self.callback_update:
            self.img = img
        else:
            self.tmp_img = img

    def timer_callback(self, timer):
        if self.img is not None:
            self.img.header.stamp = rospy.Time.now()
            self.pub.publish(self.img)

    def set_image(self, req):
        self.img = self.tmp_img
        return TriggerResponse(True, 'set image')


if __name__ == '__main__':
    rospy.init_node('image_place_holder', anonymous=False)
    ImagePlaceHolder()
    rospy.spin()
