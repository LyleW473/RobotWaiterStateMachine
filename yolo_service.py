import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2
from yolov4 import Detector
import random
# TODO add here missing imports from ros messages such as Image, YOLODetection, etc.
from sensor_msgs.msg import Image
from second_coursework.srv import YOLOLastFrame, YOLOLastFrameResponse
from second_coursework.msg import YOLODetection

class YOLOService:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.colors = {}

        # TODO ADD HERE YOUR CODE TO INITIALIZE THE DETECTOR, SUBSCRIBERS AND SERVICES
        self.camera_subscriber = rospy.Subscriber("/camera/image", Image, self.receive_raw_image)
        self.yolo_service = rospy.Service("/detect_food", YOLOLastFrame, self.execute_service)
        self.detector = Detector(
                                 gpu_id=0,  # CPU
                                 config_path="/opt/darknet/cfg/yolov4.cfg",
                                 weights_path="/opt/darknet/yolov4.weights",
                                 lib_darknet_path="/opt/darknet/libdarknet.so",
                                 meta_path="/home/k22039642/ros_ws2/src/second_coursework/config/coco.data"
                                 )

        self.detection_publisher = rospy.Publisher("/food_detections", YOLOLastFrameResponse, queue_size=1)

    def receive_raw_image(self, msg):
        # TODO ADD HERE YOUR CODE
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def execute_service(self, request):
        response = YOLOLastFrameResponse()
        rospy.loginfo("Executing service")

        if self.cv_image is None:
            rospy.logwarn("No image received yet!")
        else:
            rospy.loginfo("Generating detections")

            # Resize image based on model parameters
            img_arr = cv2.resize(self.cv_image, (self.detector.network_width(), self.detector.network_height()))
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
            cv_height, cv_width, _ = self.cv_image.shape

            for detection in detections:
                d = YOLODetection(detection.class_name, detection.class_confidence, detection.left_x, detection.top_y,
                                  detection.width, detection.height)
                #
                # print(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} %')

                # Create and publish response
                response.detections.append(d)
                response.class_prediction = detection.class_name
                response.class_confidence = detection.class_confidence

                rospy.loginfo(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} %')
                self.detection_publisher.publish(response)

        return response