#!/usr/bin/env python3
import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from utils.general import non_max_suppression, scale_coords
from utils.plots import plot_one_box

class SodaDetect:
    def __init__(self):
        rospy.init_node('soda_detection')
        model_path = '/home/vlois/catkin_ws/src/intro_robo/robotics-final-project/yolov7/runs/train/exp5/weights/best.pt'
        self.device = 'cpu'
        self.model = torch.load(model_path, map_location=self.device)
        self.model.to(self.device)
        self.model.eval()

        self.labels = ["7up", "Coke", "Pepsi", "Sprite", "Big", "Est", "Fanta"]

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/detection_image', Image, queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return
        
        input_image = self.prepare_image(cv_image)
        with torch.no_grad():
            results = self.model(input_image)
            results = non_max_suppression(results, 0.4, 0.5)

        processed_results = self.process_results(results, cv_image)
        rospy.loginfo("Detected Soda: %s", processed_results)

        
        try:
            detection_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(detection_msg)
        except CvBridgeError as e:
            rospy.logerr("Failed to change image to ROS message: {0}".format(e))

    def prepare_image(self, cv_image, img_w = 240, img_h=320):
        cv_image = cv2.resize(cv_image, (img_w, img_h))
        img = cv_image[:, :, ::-1].transpose(2, 0, 1)
        img = torch.from_numpy(img).float() / 255.0
        img = img.unsqueeze(0)
        img = img.to(self.device)
        return img

    def process_results(self, results, cv_image):
        output = []
        for det in results[0]: 
            if len(det):
                for *xyxy, conf, cls in det:
                    label = f"{self.labels[int(cls)]} {conf:.2f}"
                    plot_one_box(xyxy, cv_image, label=label, color=(255, 0, 0), line_thickness=3)
                    output.append(label)
        return output

if __name__ == '__main__':
    detector = SodaDetect()
    rospy.spin()
