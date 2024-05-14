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
    "orange": [np.array([0,35,204]), np.array([14,101,229])],
    "blue": [np.array([0,0,0]), np.array([0,0,0])]
}

# mapping of targets to nodes in map
# TODO: supply these
target_nodes = [None, None, None]

# robot action order goes:
# await_action (waiting for robot to receive next object/target)
# calculate_obj_path (perform A* from robot's current position to object area)
# pursue_obj_area (follow calculated path to object area)
## three substates:
### wait_for_camera (have camera send first movement command; transition to drive_straight immediately after)
### drive_straight (for either driving forward or backward)
### turning (for turning left or right)
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
        self.direction = None
        self.path = []
        self.path_index = 0
        self.move_backward = False
        self.state = "await_action"
        self.twist = Twist()

        # read adjacency matrix file to initialize additional matrices
        self.init_matrices("adjacency.csv")


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

        #rospy.spin()
        self.main_loop()


    # initialize adjacency and direction matrices from file
    def init_matrices(self, filename):
        # read file matrix into object
        with open(filename, 'r') as file:
            reader = csv.reader(file)
            matrix = list(reader)

        # initialize adjacency and direction matrices
        self.adj_matrix = []
        self.dir_matrix = []
        
        # iterate through every row in file matrix
        for row in matrix:
            adj_row = []
            dir_row = []

            # iterate through every cell in row
            for cell in row:
                # if cell is empty, append invalid values to adj/dir rows
                if cell == '[]':
                    adj_row.append(float('inf'))
                    dir_row.append(float('inf'))
                # otherwise, parse cell values
                else:
                    # convert string "[distance, direction]" into values
                    cell = cell[1:-1]
                    distance, direction = cell.split(",")
                    adj_row.append(float(distance))
                    dir_row.append(int(direction))
            
            # append adj/dir rows to matrices
            self.adj_matrix.append(adj_row)
            self.dir_matrix.append(dir_row)


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
        # perform action only when state is drive_straight
        if self.state == "drive_straight":
            # process image in search of path pixels
            masked_image = self.process_for_path(data)

            # use moments() function to find center of path pixels
            M = cv2.moments(masked_image)

            # check if there were any path pixels
            if M['m00'] > 0:
                # center of detected pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # make bot turn depending on how far away center of pixels are from center
                e_t = (w // 2) - cx
                k_p = 0.3 / (w // 2)

                # invert the correction if bot is driving backwards
                if self.drive_backward:
                    self.twist.angular.z = -e_t * k_p
                else:
                    self.twist.angular.z = e_t * k_p

                # check once again to make sure robot hasn't transitioned back to pursue_obj_area
                if self.state != "drive_straight":
                    # if it has, cancel the movement
                    self.vel_pub.publish(Twist())

                # otherwise, make robot move as normal
                else:
                    self.vel_pub.publish(twist)


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
        mask[search_top:h, 0:left] = 0
        mask[search_top:h, right:w] = 0

        return mask


    # perform dijkstra's algorithm from self.position to destination
    def find_path(self, destination):
        # initialize arrays for dijkstra's algorithm
        source = self.position
        n = len(self.adj_matrix)
        dist = [float('inf')] * n
        dist[source] = 0
        visited = [False] * n
        predecessor = [-1] * n

        # perform dijkstra's algorithm
        for _ in range(n):
            # find vertex with the minimum distance from the set of vertices not yet visited
            u = min((v for v in range(n) if not visited[v]), key=lambda v: dist[v])
            visited[u] = True

            # update distance of the adjacent vertices of the selected vertex
            for v in range(n):
                if self.adj_matrix[u][v] > 0 and not visited[v] and dist[v] > dist[u] + self.adj_matrix[u][v]:
                    dist[v] = dist[u] + self.adj_matrix[u][v]
                    predecessor[v] = u

        # reconstruct the shortest path from source to destination using the predecessor array
        path = []
        step = destination
        while step != -1:
            # add current step to array and backtrack w/predecessor array
            path.append(step)
            step = predecessor[step]

            # if backtrack leads to source, end loop
            if step == source:
                path.append(step)
                break
        
        # update self.path with the path array reversed so robot can follow it to destination array
        self.path = path[::-1]
        self.path_index = 0

        # after path is calculated, transition to pursue_obj_area state
        self.state = "pursue_obj_area"
        return


    # main thread loop, used for managing time for driving straight and turning
    def main_loop(self):
        while True:
            # if state is pursue_obj_area, calculate target and transition state accordingly
            if self.state == "pursue_obj_area":
                # determine next action
                action = self.get_next_action()
                
                # if robot should drive forward or back, update state accordingly
                if action == "forward" or action == "back":
                    # update twist linear velocity for camera function
                    self.move_backward = action == "back"
                    if self.move_backward:
                        self.twist.linear.x = -0.1
                    else:
                        self.twist.linear.x = 0.1
                    self.twist.angular.z = 0
                    
                    # TODO: make sure this calculation is correct
                    # since distances are reported in centimeters, also report speed in cm/s
                    next_node = self.path[self.path_index + 1]
                    dist = self.adj_matrix[self.position][next_node]
                    assert(dist != float('inf'))
                    time_to_wait = dist / 10

                    # transition state to drive_straight to allow camera to process orange pixels
                    self.state = "drive_straight"

                    # drive forward
                    self.vel_pub.publish(self.twist)

                    # sleep so robot can drive to target in distance
                    rospy.sleep(time_to_wait)

                    # stop robot
                    self.state = "pursue_obj_area"
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0
                    self.vel_pub.publish(self.twist)

                    # check if robot has reached end of path
                    if self.path_index == len(self.path) - 1:
                        self.state = "obj_turn_left"
                    # otherwise, update robot position and continue along path
                    else:
                        # update robot position
                        self.path_index += 1
                        self.position = self.path[self.path_index]

                        # transition state back to pursue_obj_area to reenter loop
                        self.state = "pursue_obj_area"

                # if robot should turn left or right, do so here
                elif action == "left" or action == "right":
                    # TODO: write left-right turning code
                    pass


    # using current position+rotation, determine whether bot should drive straight (forward or back)
    # or turn left/right
    def get_next_action(self):
        # make sure robot position and node at path index are the same
        assert(self.position == self.path[self.path_index])

        # init some variables
        cur_node = self.position
        cur_dir = self.direction

        # get next node in path and direction needed to get there
        next_node = self.path[self.path_index + 1]
        next_dir = self.dir_matrix[cur_node][next_node]

        # compare current direction and necessary direction
        # if current and necessary direction are the same, robot should drive forward
        if self.direction == next_dir:
            return "forward"
        # if current and necessary direction are 180 deg apart, robot should drive backwards
        elif abs(self.direction - next_dir) == 180:
            return "back"
        # if necessary direction is 90 degrees from current direction, robot should turn right
        elif (self.direction + 90) % 360 == next_dir:
            return "right"
        # otherwise, turn left
        else:
            return "left"

if __name__ == "__main__":
    node = RoboCourrier()

