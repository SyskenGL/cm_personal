#!/usr/bin/env python3
import rospy
from cm_driver.cm_camera import CMCamera


if __name__ == "__main__":
    cm_camera = CMCamera()
    cm_camera.start()
    rospy.spin()
