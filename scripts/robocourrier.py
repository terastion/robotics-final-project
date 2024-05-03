#!/usr/bin/env python3

import rospy, cv2, cv_bridge, moveit_commander
import numpy as np
import math
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
#from std_msgs.msg import Empty

class RoboCourrier(object):
    def __init__(self):
        # initialize this node
        rospy.init_node("robocourrier")

        # establish /camera/rgb/image_raw subscriber
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.handle_image)
        self.bridge = cv_bridge.CvBridge()

        # establish /scan subscriber
        rospy.Subscriber("/scan", LaserScan, self.handle_scan)

        # establish /cmd_vel publisher
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # establish interface for group of joints making up robot arm
        self.robot_arm = moveit_commander.MoveGroupCommander("arm")

        # establish interface for group of joints making up robot gripper
        self.robot_gripper = moveit_commander.MoveGroupCommander("gripper")

        rospy.spin()

    # handler for scanner
    def handle_scan(self, data):
        # TODO
        pass

    # handler for rbpi camera
    def handle_image(self, data):
        # TODO
        pass

if __name__ == "__main__":
    node = RoboCourrier()

