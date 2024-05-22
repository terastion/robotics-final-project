#!/usr/bin/env python

import rospy
import torch
from torchvision import transforms, models
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from torchvision.models import resnet18
import torch.nn as nn
from PIL import Image as PILImage
import os
import cv2

class SodaClassifier:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = self.load_model()
        
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])
        
        self.labels = ['Coke', 'Pepsi', 'Sprite']
        self.class_name = None
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.class_pub = rospy.Publisher('/soda_classification', String, queue_size=10)

    def load_model(self):
        #model = torch.load('soda_classifier.pt')
        #model.eval()
        model = resnet18(pretrained = True)
        num_ftrs = model.fc.in_features
        model.fc = nn.Linear(num_ftrs, 3)  
        model.load_state_dict(torch.load('/home/vlois/catkin_ws/src/intro_robo/robotics-final-project/scripts/soda_classifier.pt'))
        model.eval()
        return model
        

    def image_callback(self, data):
        
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        color_converted = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(color_converted)
        print("showing cam")
        cv2.imshow("window", cv_image)
        cv2.waitKey(3)
        input_image = self.transform(pil_image).unsqueeze(0)
        with torch.no_grad():
            outputs = self.model(input_image)
            _, preds = torch.max(outputs, 1)
            self.class_name = self.labels[preds]
            self.class_pub.publish(self.class_name)
            rospy.loginfo(f'Sodas: {self.class_name}')

if __name__ == '__main__':
    rospy.init_node('soda_classifier', anonymous=True)
    sc = SodaClassifier()
    rospy.spin()
