#!/usr/bin/env python3
import rospy
import cv2
from cm_driver.cm_node import CMNode
from sensor_msgs.msg import Image


class CMCamera(CMNode):

    def __init__(self, name='cm_camera'):
        CMNode.__init__(self, name)
        self.__cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.__cap.isOpened():
            rospy.logerr('Unable to open the camera.')
            exit(1)
        # self.__pub_topic_image = rospy.Publisher('~image', Image, queue_size=10)

    def on_ros_shutdown(self):
        if self.__cap.isOpened():
            self.__cap.release()

    def run(self):
        while self.is_running():
            pass