#!/usr/bin/env python3
import os
import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from utils.ipm import homography, warp_to_ground
from utils.filters import HoldTrue

class OppLaneIPM(DTROS):
    def __init__(self, node_name):
        super(OppLaneIPM, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        self._vehicle_name = os.environ['VEHICLE_NAME']
        p = rospy.get_param
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(f"/{self._vehicle_name}/opp_lane_free", Bool, queue_size=1)

        self.low = np.array(p("mask/hsv/low", [0,40,40]), np.uint8)
        self.high = np.array(p("mask/hsv/high", [30,255,255]), np.uint8)
        self.min_area = int(p("mask/min_area_px", 350))
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                (int(p("mask/morph_kernel",3)),)*2)

        src_pts = p("ipm/src_pts")
        dst_pts = p("ipm/dst_pts")
        self.H = homography(src_pts, dst_pts)
        self.hold = HoldTrue(hold_s=float(p("opp_clear_hold_s",0.5)))

        # Precompute opposite-lane ROI in warped image pixels (roughly)
        self.roi_px = np.array([[260,60],[360,60],[360,220],[260,220]], np.int32)

        cam_topic = p("topics/camera", f"/{self._vehicle_name}/camera_node/image/compressed")
        # Check if topic is compressed or raw based on topic name
        if "compressed" in cam_topic:
            self.sub = rospy.Subscriber(cam_topic, CompressedImage, self.cb_compressed, queue_size=1)
        else:
            self.sub = rospy.Subscriber(cam_topic, Image, self.cb, queue_size=1)

    def cb_compressed(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return
        self.process_image(img)

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.process_image(img)

    def process_image(self, img):
        warped = warp_to_ground(img, self.H, out_size=(480,360))
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.low, self.high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        # Count area in ROI
        roi_mask = np.zeros_like(mask)
        cv2.fillPoly(roi_mask, [self.roi_px], 255)
        area = cv2.countNonZero(cv2.bitwise_and(mask, roi_mask))
        opp_free_instant = (area < self.min_area)
        opp_free = self.hold.update(opp_free_instant)
        self.pub.publish(Bool(opp_free))

if __name__ == "__main__":
    node = OppLaneIPM(node_name='ipm_opp_lane')
    rospy.spin()
