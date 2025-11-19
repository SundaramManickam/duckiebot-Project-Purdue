#!/usr/bin/env python3
import os
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList, Segment
from std_msgs.msg import Bool
from duckietown.dtros import DTROS, NodeType


class IntersectionDetector(DTROS):
    def __init__(self, node_name):
        super(IntersectionDetector, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Subscriber to line detector segments
        self.sub_segments = rospy.Subscriber(
            f"/{self._vehicle_name}/line_detector_node/segment_list",
            SegmentList,
            self.cb_segments,
            queue_size=1
        )

        # Publisher for intersection detection
        self.pub_intersection = rospy.Publisher(
            f"/{self._vehicle_name}/intersection_detected",
            Bool,
            queue_size=1
        )

    def cb_segments(self, msg):
        # Look for RED segments
        # Segment color constants: WHITE=0, YELLOW=1, RED=2
        
        red_segments_count = 0
        for segment in msg.segments:
            if segment.color == Segment.RED:
                red_segments_count += 1
        
        # If we see enough red segments, we assume it's an intersection tape
        if red_segments_count > 2:
            self.pub_intersection.publish(Bool(True))
        else:
            self.pub_intersection.publish(Bool(False))

if __name__ == "__main__":
    node = IntersectionDetector(node_name='intersection_detector')
    rospy.spin()
