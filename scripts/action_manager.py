#!/usr/bin/env python3

import rospy
from robocourier.msg import RobotAction
from std_msgs.msg import Empty

def to_index(node_name):
    if len(node_name) == 2:
        return 26 + ord(node_name.lower()[1])-97
    elif len(node_name) == 1:
        return ord(node_name.lower())-97

class ActionManager(object):

    def __init__(self):
        # initialize this node
        rospy.init_node('action_manager')

        # list of actions to take
        # may get updated with additional node for receiver
        self.actions = [
            {"object": "one", "tag": 1},
            {"object": "two", "tag": 2},
            {"object": "three", "tag": 3}
        ]
        self.action_idx = 0

        # establish /robocourier/state subscriber for receiving pings from robot
        rospy.Subscriber("/robocourier/state", Empty, self.determine_action)

        # establish /robocourier/robot_action publisher for sending actions to robot
        self.action_pub = rospy.Publisher("/robocourier/robot_action", RobotAction, queue_size=10)
        print("action manager initialized")

        self.run()

    # callback function for receiving a ping from robot
    def determine_action(self, data):
        # check if index isn't out of range
        if self.action_idx < 3:
            # create action message
            current_action = self.actions[self.action_idx]
            msg = RobotAction(obj=current_action["object"], tag=current_action["tag"])
            self.action_pub.publish(msg)

            self.action_idx += 1
        # otherwise, send an empty message
        else:
            self.action_pub.publish(RobotAction())

    def run(self):
        rospy.spin()

if __name__=="__main__":

    node = ActionManager()
