#!/usr/bin/env python3

import numpy as np
import rospy
from cv_bridge import CvBridge

from dt_state_estimation.lane_filter import LaneFilterHistogram
from dt_state_estimation.lane_filter.types import (
    Segment,
    SegmentPoint,
    SegmentColor,
)
from dt_state_estimation.lane_filter.rendering import plot_belief, plot_d_phi
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import LanePose, SegmentList, WheelEncoderStamped, EpisodeStart
from duckietown_msgs.msg import Segment as SegmentMsg
from sensor_msgs.msg import CompressedImage



class LaneFilterNode(DTROS):
    """Generates an estimate of the lane pose.

    Creates a `lane_filter` to get estimates on `d` and `phi`, the lateral and heading deviation from the
    center of the lane.
    It gets the segments extracted by the line_detector as input and output the lane pose estimate.


    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~filter (:obj:`list`): A list of parameters for the lane pose estimation filter
        ~debug (:obj:`bool`): A parameter to enable/disable the publishing of debug topics and images

    Subscribers:
        ~segment_list (:obj:`SegmentList`): The detected line segments from the line detector
        ~(left/right)_wheel_encoder_node/tick (:obj: `WheelEncoderStamped`): Information from the wheel encoders\
        ~episode_start (:obj: `EpisodeStart`): The signal that a new episode has started - used to reset the filter

    Publishers:
        ~lane_pose (:obj:`LanePose`): The computed lane pose estimate
        ~debug/belief_img/compressed (:obj:`CompressedImage`): A debug image that shows the filter's internal state
        ~seglist_filtered (:obj:``SegmentList): a debug topic to send the filtered list of segments that
        are considered as valid

    """

    filter: LaneFilterHistogram
    bridge: CvBridge

    def __init__(self, node_name):
        super(LaneFilterNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION,
            #fsm_controlled=True
        )

        self._filter = rospy.get_param("~lane_filter_histogram_configuration", None)
        #self._debug = rospy.get_param("~debug", False)
        self._debug = True
        self._predict_freq = rospy.get_param("~predict_frequency", 30.0)
        #Enocder Init
        self.right_encoder_ticks = 0
        self.right_encoder_initialized = False
        self.left_encoder_ticks = 0
        self.left_encoder_initialized = False
        self.right_encoder_ticks_delta = 0
        self.left_encoder_ticks_delta = 0


        # Load the needed filter parameters defined elsewhere need here
        if self._filter is None:
            rospy.logerr("[Lane filter] lane_filter_histogram_configuration parameter not found!")
            raise ValueError("lane_filter_histogram_configuration parameter is required")
        
        try:
            self._filter['encoder_resolution'] = rospy.get_param("left_wheel_encoder_driver_node/resolution", 135)
            self._filter['wheel_baseline'] = rospy.get_param("kinematics_node/baseline")
            self._filter['wheel_radius'] = rospy.get_param("kinematics_node/radius")
        except (rospy.KeyError, TypeError) as e:
            rospy.logerror(f"[Lane filter] Unable to load required param: {e}")
            raise  # Re-raise to prevent initialization with incomplete parameters

        # Create the filter
        try:
            self.filter = LaneFilterHistogram(**self._filter)
            rospy.loginfo("[Lane filter] LaneFilterHistogram initialized successfully")
        except Exception as e:
            rospy.logerr(f"[Lane filter] Failed to initialize LaneFilterHistogram: {e}")
            raise  # Re-raise to prevent node from running with uninitialized filter


        # this is only used for the timestamp of the first publication
        self.last_update_header = None


        # Creating cvBridge
        self.bridge = CvBridge()


        # Publishers - Create these BEFORE subscribers to ensure they exist when callbacks are triggered
        try:
            self.pub_lane_pose = rospy.Publisher(
                "~lane_pose", LanePose, queue_size=1, dt_topic_type=TopicType.PERCEPTION
            )
            rospy.loginfo("[Lane filter] pub_lane_pose publisher initialized")
        except Exception as e:
            rospy.logerr(f"[Lane filter] Failed to initialize pub_lane_pose: {e}")
            self.pub_lane_pose = None

        try:
            self.pub_belief_img = rospy.Publisher(
                 "~debug/belief_img/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
            )
        except Exception as e:
            rospy.logwarn(f"[Lane filter] Failed to initialize pub_belief_img: {e}")
            self.pub_belief_img = None

        try:
            self.pub_plot_d_phi = rospy.Publisher(
                "~debug/plot_d_phi/compressed", CompressedImage, queue_size=1, dt_topic_type=TopicType.DEBUG
            )
        except Exception as e:
            rospy.logwarn(f"[Lane filter] Failed to initialize pub_plot_d_phi: {e}")
            self.pub_plot_d_phi = None


        # Subscribers - Create these AFTER publishers to avoid callbacks before publishers exist

        self.sub_segment_list = rospy.Subscriber(
            "~segment_list", SegmentList, self.cbProcessSegments, queue_size=1
        )
        rospy.loginfo(f"[Lane filter] Subscribed to: {self.sub_segment_list.resolved_name}")

        self.sub_encoder_left = rospy.Subscriber(
            "~left_wheel_encoder_driver_node/tick", WheelEncoderStamped, self.cbProcessLeftEncoder, queue_size=1
        )

        self.sub_encoder_right = rospy.Subscriber(
            "~right_wheel_encoder_driver_node/tick", WheelEncoderStamped, self.cbProcessRightEncoder, queue_size=1
        )



        # Set up a timer for prediction (if we got encoder data) since that data can come very quickly
  #      rospy.Timer(rospy.Duration(1 / self._predict_freq), self.cbPredict)
        # Don't publish estimate during initialization - wait for first segment message
        # self.publishEstimate(self.last_update_header)


    def cbEpisodeStart(self, msg):
        rospy.loginfo("Lane Filter Resetting")
        self.filter.initialize_belief()

    @staticmethod
    def _seg_msg_to_custom_type(msg: SegmentMsg):
        color: SegmentColor = SegmentColor.WHITE
        if msg.color == SegmentMsg.YELLOW:
            color = SegmentColor.YELLOW
        elif msg.color == SegmentMsg.RED:
            color = SegmentColor.RED

        p1, p2 = msg.points

        return Segment(
            color=color,
            points=[
                SegmentPoint(x=p1.x, y=p1.y),
                SegmentPoint(x=p2.x, y=p2.y),
            ],
        )

    def cbProcessLeftEncoder(self, left_encoder_msg):
        # we need to account for the possibility that the encoder is not reading
        # 0 at startup
        if not self.left_encoder_initialized:
            self.left_encoder_ticks = left_encoder_msg.data
            self.left_encoder_initialized = True
        self.left_encoder_ticks_delta = left_encoder_msg.data - self.left_encoder_ticks

    def cbProcessRightEncoder(self, right_encoder_msg):
        if not self.right_encoder_initialized:
            self.right_encoder_ticks = right_encoder_msg.data
            self.right_encoder_initialized = True
        self.right_encoder_ticks_delta = right_encoder_msg.data - self.right_encoder_ticks

    def cbPredict(self):
        if self.left_encoder_ticks_delta == 0 or self.right_encoder_ticks_delta == 0:
            return
        self.filter.predict(self.left_encoder_ticks_delta, self.right_encoder_ticks_delta)
        self.left_encoder_ticks += self.left_encoder_ticks_delta
        self.right_encoder_ticks += self.right_encoder_ticks_delta
        self.left_encoder_ticks_delta = 0
        self.right_encoder_ticks_delta = 0

        self.publishEstimate(self.last_update_header)

    def cbProcessSegments(self, segment_list_msg):
        """Callback to process the segments

        Args:
            segment_list_msg (:obj:`SegmentList`): message containing list of processed segments

        """
        # Debug: Log when segments are received
        num_segments = len(segment_list_msg.segments)
        rospy.loginfo_throttle(2.0, f"[Lane filter] Received {num_segments} projected segments")
        
        if num_segments == 0:
            rospy.logwarn_throttle(5.0, "[Lane filter] Received 0 segments - cannot update filter")
            return
        
        self.cbPredict()
        self.last_update_header = segment_list_msg.header
        dt_segment_list = []
        # we need to parse the data in the ROS data struct and port into a dt data struct
        for segment in segment_list_msg.segments:
            dt_segment_color = None
            if segment.color == SegmentMsg.WHITE:
                dt_segment_color = SegmentColor.WHITE
            elif segment.color == SegmentMsg.YELLOW:
                dt_segment_color = SegmentColor.YELLOW
            elif segment.color == SegmentMsg.RED:
                dt_segment_color = SegmentColor.RED

            dt_points = []
            for point in segment.points:
                dt_point = SegmentPoint(x=point.x, y=point.y)
                dt_points.append(dt_point)

            dt_segment = Segment(points=dt_points, color=dt_segment_color)
            dt_segment_list.append(dt_segment)


        # Debug: Log filter update
        rospy.loginfo_throttle(2.0, f"[Lane filter] Updating filter with {len(dt_segment_list)} segments")
        
        self.filter.update(dt_segment_list)
        
        # Debug: Log estimate after update
        [d, phi] = self.filter.get_estimate()
        rospy.loginfo_throttle(2.0, f"[Lane filter] Filter estimate: d={d:.3f}, phi={phi:.3f}")

        self.publishEstimate(segment_list_msg.header)

    def publishEstimate(self, header):
        # Don't publish if header is None
        if header is None:
            return

        [d_max, phi_max] = self.filter.get_estimate()

        # Getting the highest belief value from the belief matrix
        max_val = self.filter.get_max()
        # Comparing it to a minimum belief threshold to make sure we are certain enough of our estimate
        in_lane = max_val > self.filter.min_max

        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header = header
        lanePose.d = d_max
        lanePose.phi = phi_max
        lanePose.in_lane = in_lane
        # XXX: is it always NORMAL?
        lanePose.status = lanePose.NORMAL

        # Check if publisher exists before using it
        if hasattr(self, 'pub_lane_pose') and self.pub_lane_pose is not None:
            self.pub_lane_pose.publish(lanePose)
        else:
            rospy.logwarn("[Lane filter] pub_lane_pose publisher not initialized, cannot publish lane pose")
        if self._debug:
            self.debugOutput()

    def debugOutput(self):
        """Creates and publishes debug messages

        """
        if self._debug:
            # Create belief image and publish it
             # LP : this is too heavy for now: (a) should be offloaded onto base station (b) should be faster
            # belief_img = self.bridge.cv2_to_compressed_imgmsg(
            # plot_belief(self.filter, dpi=30)
            #)
            #self.pub_belief_img.publish(belief_img)

            # Check if publisher exists before using it
            if hasattr(self, 'pub_plot_d_phi'):
                try:
                    d_max, phi_max = self.filter.get_estimate()
                    plot_d_phi_img = self.bridge.cv2_to_compressed_imgmsg(
                        plot_d_phi(d=d_max, phi=phi_max)
                    )
                    self.pub_plot_d_phi.publish(plot_d_phi_img)
                except Exception as e:
                    rospy.logwarn(f"[Lane filter] Error publishing debug plot_d_phi: {e}")


    def loginfo(self, s):
        rospy.loginfo("[%s] %s" % (self.node_name, s))

if __name__ == "__main__":
    lane_filter_node = LaneFilterNode(node_name="lane_filter_node")
    rospy.spin()
