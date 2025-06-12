import cv2
import random
import numpy as np
from typing import Tuple
import sys, os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, BoundingBox2D, Detection
from std_msgs.msg import String

from .lib import camera_perception_func_lib as CPFL

# ---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_IMAGE_TOPIC_NAME = "image_raw"

# Publish할 토픽 이름
PUB_TOPIC_NAME = "yolov8_traffic_light_info"

# ----------------------------------------------

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector_node')

        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_image_topic = self.declare_parameter('sub_image_topic', SUB_IMAGE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        self.cv_bridge = CvBridge()

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.detection_sub = Subscriber(self, DetectionArray, self.sub_detection_topic, qos_profile=self.qos_profile)
        self.image_sub = Subscriber(self, Image, self.sub_image_topic, qos_profile=self.qos_profile)
        self.ts = ApproximateTimeSynchronizer([self.detection_sub, self.image_sub], queue_size=1, slop=0.5)
        self.ts.registerCallback(self.sync_callback)

        self.publisher = self.create_publisher(String, self.pub_topic, self.qos_profile)

    def sync_callback(self, detection_msg: DetectionArray, image_msg: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg)
        
        traffic_light_detected = False
        for detection in detection_msg.detections:
            if detection.class_name == 'traffic_light':

                hsv_ranges = {
                    'red1': (np.array([0, 100, 95]), np.array([10, 255, 255])),
                    'red2': (np.array([160, 100, 95]), np.array([179, 255, 255])),
                    'yellow': (np.array([20, 100, 95]), np.array([30, 255, 255])),
                    'green': (np.array([40, 100, 95]), np.array([90, 255, 255]))
                }

                # get_traffic_light_color -> Red, Yellow, Green, Unknown
                traffic_light_color = CPFL.get_traffic_light_color(cv_image, detection.bbox, hsv_ranges) 
                
                # Publish traffic light color as string
                color_msg = String()
                color_msg.data = traffic_light_color
                print(f'traffic light: {color_msg.data}') 
                self.publisher.publish(color_msg)
                traffic_light_detected = True
                break  # Only process the first detected traffic light

        if not traffic_light_detected:
            # Publish 'None' if no traffic light is detected
            color_msg = String()
            color_msg.data = 'None'
            print(f'traffic light: {color_msg.data}')
            self.publisher.publish(color_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
