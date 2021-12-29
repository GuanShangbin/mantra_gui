#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class pyTranslator:
    def __init__(self):
        rospy.init_node('yolo_reader', anonymous=True)
        try:
            self.camera_sub = rospy.Subscriber('/darknet_ros/detection_image', Image, self.camera_callback)
            print('subscribed')
            rospy.spin()
        except:
            rospy.signal_shutdown('节点退出')

    def camera_callback(self, image):
        image = CvBridge().imgmsg_to_cv2(image, 'bgr8')
        cv2.imwrite('../../../devel/lib/ebox_gui/camera.png',image)
        print('image writed!')

pyTranslator()
