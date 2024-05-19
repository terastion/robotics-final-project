#!/usr/bin/env python3
import rospy
import cv2, cv_bridge
import torch
#import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from utils.general import non_max_suppression, scale_coords
from utils.plots import plot_one_box
from models.yolo import Model
from torch.profiler import profile, record_function, ProfilerActivity

import os
from pathlib import Path

class SodaDetect:
    def __init__(self):
        rospy.init_node('soda_detection')
        current_directory = Path(__file__)
        parent_directory = current_directory.parent
        print(parent_directory)
        self.device = 'cpu'
        #self.model = torch.hub.load(str(parent_directory), 'custom', 'best.pt', source= 'local' )
        #self.model.to(self.device)
        yaml_file = parent_directory / 'cfg/training/yolov7.yaml'
        best_file = parent_directory / 'best.pt'
        self.model = Model(str(yaml_file))
        self.model.load_state_dict(torch.load(str(best_file)), strict=False)
        self.model.eval()

        self.labels = ["Coke", "Pepsi", "Sprite"]
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/detection_image', Image, queue_size=10)
        inputs = torch.randn(5, 3, 224, 224)
        with profile(
        activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA],
        with_stack=True,
    ) as prof:
            self.model(inputs)

        # Print aggregated stats
        print(prof.key_averages(group_by_stack_n=5).table(sort_by="self_cuda_time_total", row_limit=2))
        

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding ="bgr8")
        print("showing cam")
        cv2.imshow("window", image)
        cv2.waitKey(3)
        
        input_image = self.prepare_image(image)
        with torch.no_grad():
            results = self.model(input_image)[0]
            print(len(results))
            results = non_max_suppression(results, 0.4, 0.5)

        processed_results = self.process_results(results, image)
        rospy.loginfo("Detected Soda: %s", processed_results)
        
        detection_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.image_pub.publish(detection_msg)
        

    def prepare_image(self, cv_image, img_w = 320, img_h=320):
        cv_image = cv2.resize(cv_image, (img_w, img_h))
        img = cv_image[:, :, ::-1].transpose(2, 0, 1)
        img = img.copy()
        img = torch.from_numpy(img)
        img = img.float() / 255.0
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
