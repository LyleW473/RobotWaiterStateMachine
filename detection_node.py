#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2
from yolov4 import Detector
import random
# TODO add here missing imports from ros messages such as Image, YOLODetection, etc.
from sensor_msgs.msg import Image
from second_coursework.msg import YOLODetection, YOLOLastDetectPrediction

class DetectionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.colors = {}

        self.detector = Detector(
                                 gpu_id=0,  # CPU
                                 config_path="/opt/darknet/cfg/yolov4.cfg",
                                 weights_path="/opt/darknet/yolov4.weights",
                                 lib_darknet_path="/opt/darknet/libdarknet.so",
                                 meta_path="/home/k22039642/ros_ws2/src/second_coursework/config/coco.data"
                                 )

        self.detections_publisher = rospy.Publisher("/food_detections", YOLOLastDetectPrediction, queue_size=1)
        self.camera_subscriber = rospy.Subscriber("/camera/image", Image, self.process_raw_image)
        self.last_recorded_time = rospy.get_time()

    def process_raw_image(self, msg):
        raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if raw_image is None:
            rospy.logwarn("No image received yet!")
        else:
            current_time = rospy.get_time()
            # If 2.5 seconds have passed (change rate of images)
            if current_time - self.last_recorded_time > 2.5:
                self.last_recorded_time = current_time

                raw_image = cv2.resize(raw_image, (self.detector.network_width(), self.detector.network_height()))
                # Increase brightness to allow for clearer detections
                self.cv_image = cv2.convertScaleAbs(raw_image, alpha=0.95, beta=30)

                # Generate detection based on raw image
                self.generate_detection()

    def generate_detection(self):
        response = YOLOLastDetectPrediction()
        rospy.loginfo("Generating detections")

        # Generate detections
        detections = self.detector.perform_detect(image_path_or_buf=self.cv_image, show_image=True)
        cv_height, cv_width, _ = self.cv_image.shape
        for detection in detections:
            d = YOLODetection(detection.class_name, detection.class_confidence, detection.left_x, detection.top_y,
                              detection.width, detection.height)
            # Create and publish response
            response.detections.append(d)
            response.class_prediction = detection.class_name
            response.class_confidence = detection.class_confidence

            rospy.loginfo(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} %')
            self.detections_publisher.publish(response)

if __name__ == "__main__":
    rospy.init_node("detections_node")
    detector = DetectionNode()

    rospy.spin()