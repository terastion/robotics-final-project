#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# Initialize the ROS node
rospy.init_node('image_capture_node')

# Create a CvBridge instance
bridge = CvBridge()

# Set the directory to save images
image_dir = 'robotics-final-project/turtlebot_images'
os.makedirs(image_dir, exist_ok=True)

# Set the number of images to capture
num_images = 500

# Define the callback function for the camera topic
def image_callback(msg):
    global image_count

    # Convert the ROS Image message to OpenCV format
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Display the image
    cv2.imshow('Camera Feed', cv_image)
    cv2.waitKey(1)

    # Capture and save the image with an incrementing filename
    image_filename = os.path.join(image_dir, f'image_pepsi5_{image_count:03d}.jpg')
    cv2.imwrite(image_filename, cv_image)
    rospy.loginfo(f'Image captured and saved: {image_filename}')

    image_count += 1
    if image_count >= num_images:
        rospy.signal_shutdown('Captured all images')

# Initialize the image count
image_count = 0

# Subscribe to the camera topic
image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

# Keep the node running
rospy.spin()