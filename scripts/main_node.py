#!/usr/bin/env python3

import rospy, cv2, cv_bridge, moveit_commander
import numpy as np
import math
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
from robocourier.msg import RangeUpdate, RobotAction
from std_msgs.msg import Empty

# HSV color ranges for tape maze
color_ranges = {
    "orange": [np.array([0,0,0]), np.array([0,0,0])],
    "blue": [np.array([0,0,0]), np.array([0,0,0])]
}

# mapping of targets to nodes in map
# TODO: supply these
target_nodes = [None, None, None]

# robot action order goes:
# await_action (waiting for robot to receive next object/target)
# calculate_obj_path (perform A* from robot's current position to object area)
# pursue_obj_area (follow calculated path to object area)
## two substates:
# obj_turn_left (turn left for first object)
###########################################
# process_obj (perform object recognition on object)
# (skip state below if object successfully detected)
# obj_turn_left (turn right for next object in case incorrect object detected)
###########################################
# grab_obj (grab object)
# calculate_target_path (perform A* from robot's current position to desired target)
# pursue_target (follow calculated path to target)
# drop_obj (place object down at target)
# (after this point, go back to await_action)

class RoboCourrier(object):
    def __init__(self):
        # initialize this node
        rospy.init_node("robocourier")

        ### ROBOT CONTROL VARIABLES ###
        # provide list of color ranges to be use/be updated
        # TODO: actually determine these values
        self.position = None
        self.state = "await_action"
        self.velocity = Twist()


        ########################################################################


        ### CONNECTIONS FOR ROBOT COMPONENTS ###
        # establish range_update subscriber
        rospy.Subscriber("/debug/range_update", RangeUpdate, self.handle_range)

        # establish /camera/rgb/image_raw subscriber
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.handle_image)
        self.bridge = cv_bridge.CvBridge()

        # establish /scan subscriber
        rospy.Subscriber("/scan", LaserScan, self.handle_scan)

        # establish /cmd_vel publisher
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # establish interface for group of joints making up robot arm
        #self.robot_arm = moveit_commander.MoveGroupCommander("arm")

        # establish interface for group of joints making up robot gripper
        #self.robot_gripper = moveit_commander.MoveGroupCommander("gripper")


        ########################################################################


        ### CONNECTIONS WITH ACTION MANAGER'S TOPICS ###
        # establish /robocourier/robot_action subscriber
        rospy.Subscriber("/robocourier/robot_action", RobotAction, self.handle_action)
        self.obj = ""
        self.tag = -1

        # establish /robocourier/state publisher
        self.state_pub = rospy.Publisher("/robocourier/state", Empty, queue_size=10)


        ########################################################################


        # send first ping message to action manager
        rospy.sleep(3)
        self.state_pub.publish(Empty())

        rospy.spin()


    # handler for action manager
    def handle_action(self, data):
        print("received action {}".format(data))
        # check for empty action, signaling completion
        if data.obj == "":
            # set object and tag to empty values
            self.obj = ""
            self.tag = -1
        # otherwise, update robot's goals and begin journey to object area
        else:
            self.obj = data.obj
            self.tag = data.tag
            
            # transition robot to next state and calculate object path
            self.state = "calculate_obj_path"
            target_node = target_nodes[data.tag]
            self.find_path(target_node)


    # handler for range updater
    def handle_range(self, data):
        self.color_ranges[data.color][int(data.is_upper)] = np.array(data.range)
        print("updated range to {}".format(data.range))


    # handler for scanner
    def handle_scan(self, data):
        # TODO
        pass


    # handler for rbpi camera
    def handle_image(self, data):
        # TODO
        self.process_for_path(data)


    # process image to see if specified color is in bottom area
    def process_for_path(self, data):
        # convert ROS message to cv2 and hsv
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # retrieve lower and upper bounds for what to consider desired color
        # hardcoded color for now
        lower_range, upper_range = self.color_ranges["orange"]

        # remove all pixels that aren't within desired color range
        mask = cv2.inRange(hsv, lower_range, upper_range)

        # limit pixel search for only pixels in center-bottom range
        h, w, d = image.shape
        middle_index = w // 2
        left = middle_index - 50
        right = middle_index + 50
        search_top = int(3*h/4)
        search_bot = int(3*h/4 + 20)
        mask[0:search_top, 0:w] = 0
        mask[search_top:search_bot, 0:left] = 0
        mask[search_top:search_bot, right:w] = 0
        mask[search_bot:h, 0:w] = 0

        # use moments() function to find center of pixels
        M = cv2.moments(mask)

        # check if there are any pixels found
        if M['m00'] > 0:
            # center of detected pixels in the image
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            # draw a red circle in the debugging window to indicate center point of pixels
            cv2.circle(image, (cx, cy), 20, (0,0,255), 1)

        # show live image feed
        cv2.imshow("window", image)
        cv2.waitKey(3)


    # perform A* algorithm from self.position to destination
    def find_path(self, destination):
        # TODO

        # after path is calculated, transition to pursue_obj_area state
        self.state = "pursue_obj_area"
        return


    # main thread loop, used for managing time for driving straight and turning
    def main_loop(self):
        # TODO
        pass


if __name__ == "__main__":
    node = RoboCourrier()

