#!/usr/bin/env python3
import os
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from duckietown.dtros import DTROS, NodeType


class TrafficLightDetector(DTROS):
    def __init__(self, node_name):
        super(TrafficLightDetector, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.bridge = CvBridge()

        # Subscribers
        self.sub_image = rospy.Subscriber(
            f"/{self._vehicle_name}/camera_node/image/compressed",
            CompressedImage,
            self.image_cb,
            queue_size=1
        )

        # Publishers
        # True = Green or No Light (Go), False = Red (Stop)
        self.pub_tl = rospy.Publisher(
            f"/{self._vehicle_name}/traffic_light_state",
            Bool,
            queue_size=1
        )
        
        # Parameters for color detection (HSV)
        # Adjust these based on your specific environment
        # Red can wrap around 0/180
        self.red_low1 = np.array([0, 100, 100])
        self.red_high1 = np.array([10, 255, 255])
        self.red_low2 = np.array([160, 100, 100])
        self.red_high2 = np.array([180, 255, 255])
        
        self.green_low = np.array([40, 100, 100])
        self.green_high = np.array([90, 255, 255])

    def image_cb(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg)
        except ValueError as e:
            return

        # Convert to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Detect Red
        mask_red1 = cv2.inRange(hsv, self.red_low1, self.red_high1)
        mask_red2 = cv2.inRange(hsv, self.red_low2, self.red_high2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        # Detect Green
        mask_green = cv2.inRange(hsv, self.green_low, self.green_high)

        # Count pixels
        red_pixels = cv2.countNonZero(mask_red)
        green_pixels = cv2.countNonZero(mask_green)
        
        # Simple logic: if significant red detected, stop. Else go.
        threshold = 200 # Minimum pixels to consider a detection
        
        is_red = red_pixels > threshold
        
        # Default to Green (True) unless Red is clearly detected
        if is_red:
            self.pub_tl.publish(Bool(False)) # Stop
        else:
            self.pub_tl.publish(Bool(True)) # Go

if __name__ == "__main__":
    node = TrafficLightDetector(node_name='traffic_light_detector')
    rospy.spin()
