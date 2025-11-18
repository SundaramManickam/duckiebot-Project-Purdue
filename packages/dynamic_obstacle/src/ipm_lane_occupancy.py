#!/usr/bin/env python3
import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from dynamic_obstacle.utils.filters import HoldTrue

class OppLaneIPM(DTROS):
    def __init__(self, node_name):
        super(OppLaneIPM, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )

        self._vehicle_name = os.environ["VEHICLE_NAME"]
        p = rospy.get_param
        self.bridge = CvBridge()

        # Publisher: is opposite lane free?
        self.pub = rospy.Publisher(
            f"/{self._vehicle_name}/opp_lane_free",
            Bool,
            queue_size=1
        )

        # HSV mask for detecting obstacles (robot / cone)
        self.low = np.array(p("mask/hsv/low",  [0, 40, 40]), np.uint8)
        self.high = np.array(p("mask/hsv/high", [30,255,255]), np.uint8)
        self.min_area = int(p("mask/min_area_px", 700))  # increased for stability
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))

        # === USE REAL CALIBRATED HOMOGRAPHY ===
        H_vals = p("calibrated_homography")
        self.H = np.array(H_vals, dtype=np.float32).reshape(3,3)

        # output (warped) image size
        self.W = 480
        self.Hh = 360

        # Predefined region in the *warped* image where the opposite lane lies
        # This rectangle works well for standard Duckietown geometry.
        self.roi_px = np.array([
            [260,  80],
            [380,  80],
            [380, 240],
            [260, 240]
        ], np.int32)

        # Debouncer for consistent readings
        self.hold = HoldTrue(hold_s=float(p("opp_clear_hold_s", 0.7)))

        cam_topic = p("topics/camera", f"/{self._vehicle_name}/camera_node/image/compressed")
        if "compressed" in cam_topic:
            self.sub = rospy.Subscriber(
                cam_topic, CompressedImage, self.cb_compressed, queue_size=1
            )
        else:
            self.sub = rospy.Subscriber(
                cam_topic, Image, self.cb_raw, queue_size=1
            )

    def cb_compressed(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is not None:
            self.process_image(img)

    def cb_raw(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.process_image(img)

    def process_image(self, img):
        # warp to top-down view using *calibrated* homography
        warped = cv2.warpPerspective(img, self.H, (self.W, self.Hh))

        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.low, self.high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        roi_mask = np.zeros_like(mask)
        cv2.fillPoly(roi_mask, [self.roi_px], 255)

        area = cv2.countNonZero(cv2.bitwise_and(mask, roi_mask))
        opp_free_inst = (area < self.min_area)

        opp_free = self.hold.update(opp_free_inst)
        self.pub.publish(Bool(opp_free))

if __name__ == "__main__":
    node = OppLaneIPM(node_name="ipm_opp_lane")
    rospy.spin()
