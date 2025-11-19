#!/usr/bin/env python3
import os
import rospy
import time
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String, Float32
from duckietown_msgs.msg import LanePose
from duckietown.dtros import DTROS, NodeType

class FSM(DTROS):
    DRIVE, STOP, AVOID = "DRIVE", "STOP", "AVOID"

    def __init__(self, node_name):
        super(FSM, self).__init__(
            node_name=node_name,
            node_type=NodeType.BEHAVIOR
        )
        self._vehicle_name = os.environ['VEHICLE_NAME']
        p = rospy.get_param
        
        # ToF threshold for obstacle detection
        self.tof_threshold = float(p("stop_dist", 0.48))
        
        # Speeds
        self.v_nominal = float(p("v_nominal", 0.25))
        self.v_avoid = float(p("v_cross", 0.18))
        
        # Avoidance parameters
        self.lat_off = float(p("lateral_offset_m", 0.15))  # Lateral offset for avoidance
        self.avoid_distance = float(p("avoid_distance_m", 0.6))  # Distance to travel during avoidance
        self.stop_duration = 1.0  # Stop for 1 second before avoiding
        
        # State variables
        self.range_m = 10.0  # ToF range in meters
        self.d = 0.0  # Lateral position from lane center (d > 0 = right, d < 0 = left)
        self.phi = 0.0  # Heading angle (phi > 0 = turning right, phi < 0 = turning left)
        self.d_history = []  # Track d values to detect if it's stuck
        self.d_history_size = 10
        
        # Subscribe to ToF sensor
        tof_topic = p("topics/tof_range", f"/{self._vehicle_name}/front_center_tof_driver_node/range")
        rospy.Subscriber(tof_topic, Range, self.tof_cb, queue_size=1)
        
        # Subscribe to lane pose to determine which side of lane we're on
        lane_pose_topic = p("topics/lane_pose", f"/{self._vehicle_name}/lane_filter_node/lane_pose")
        rospy.Subscriber(lane_pose_topic, LanePose, self.lane_pose_cb, queue_size=1)
        
        # Publishers
        self.pub_bias = rospy.Publisher(f"/{self._vehicle_name}/fsm/lateral_offset", Float32, queue_size=1)
        self.pub_speed = rospy.Publisher(f"/{self._vehicle_name}/fsm/speed_cap", Float32, queue_size=1)
        self.pub_state = rospy.Publisher(f"/{self._vehicle_name}/fsm/state", String, queue_size=1)
        
        # FSM state
        self.state = self.DRIVE
        self.t_enter = time.time()  # Time when entering current state
        self.dist_progress = 0.0  # Distance traveled in current state
        self.last_t = time.time()
        self.avoid_bias_direction = 0.0  # Direction of avoidance bias (positive = right, negative = left)

    def tof_cb(self, msg): 
        """Callback for ToF sensor - range is already in meters"""
        self.range_m = float(msg.range)
    
    def lane_pose_cb(self, msg):
        """Callback for lane pose - d tells us which side of lane we're on"""
        self.d = msg.d
        self.phi = msg.phi
        # d > 0 means robot is RIGHT of center
        # d < 0 means robot is LEFT of center
        # phi > 0 means robot is turning RIGHT
        # phi < 0 means robot is turning LEFT
        
        # Track d history to detect if it's stuck on one side
        self.d_history.append(self.d)
        if len(self.d_history) > self.d_history_size:
            self.d_history.pop(0)

    def trans(self, new):
        """Transition to new state"""
        self.state = new
        self.t_enter = time.time()
        self.dist_progress = 0.0
        self.pub_state.publish(String(self.state))
        rospy.loginfo(f"[FSM] Transitioned to state: {self.state}")

    def run(self):
        rate = rospy.Rate(30)
        self.pub_state.publish(String(self.state))
        
        while not rospy.is_shutdown():
            now = time.time()
            dt = now - self.last_t
            self.last_t = now
            
            # Default outputs
            bias = 0.0
            speed_cap = self.v_nominal
            
            # State machine logic
            if self.state == self.DRIVE:
                # Normal driving - check for obstacles
                if self.range_m < self.tof_threshold:
                    rospy.loginfo(f"[FSM] Obstacle detected at {self.range_m:.2f}m, transitioning to STOP")
                    
                    # Determine avoidance direction
                    # Check if d is stuck (always same sign) - indicates poor lane detection
                    d_stuck = False
                    if len(self.d_history) >= self.d_history_size:
                        all_negative = all(d_val < 0 for d_val in self.d_history)
                        all_positive = all(d_val > 0 for d_val in self.d_history)
                        d_stuck = all_negative or all_positive
                    
                    if d_stuck:
                        # d is stuck - use phi (heading) as fallback
                        rospy.logwarn(f"[FSM] d is stuck (all {self.d_history[-1]:.3f}), using phi as fallback")
                        if self.phi > 0.1:
                            # Turning right → avoid left
                            self.avoid_bias_direction = -self.lat_off
                            rospy.loginfo(f"[FSM] Robot turning RIGHT (phi={self.phi:.3f}), will avoid LEFT")
                        elif self.phi < -0.1:
                            # Turning left → avoid right
                            self.avoid_bias_direction = self.lat_off
                            rospy.loginfo(f"[FSM] Robot turning LEFT (phi={self.phi:.3f}), will avoid RIGHT")
                        else:
                            # phi is small - use default: if d is negative, assume left side, avoid right
                            if self.d < 0:
                                self.avoid_bias_direction = self.lat_off
                                rospy.loginfo(f"[FSM] d stuck negative, defaulting to avoid RIGHT")
                            else:
                                self.avoid_bias_direction = -self.lat_off
                                rospy.loginfo(f"[FSM] d stuck positive, defaulting to avoid LEFT")
                    else:
                        # d is working - use it normally
                        if self.d > 0:
                            # Robot is on RIGHT side → avoid LEFT (negative bias)
                            self.avoid_bias_direction = -self.lat_off
                            rospy.loginfo(f"[FSM] Robot on RIGHT side (d={self.d:.3f}), will avoid LEFT")
                        else:
                            # Robot is on LEFT side → avoid RIGHT (positive bias)
                            self.avoid_bias_direction = self.lat_off
                            rospy.loginfo(f"[FSM] Robot on LEFT side (d={self.d:.3f}), will avoid RIGHT")
                    
                    self.trans(self.STOP)
                else:
                    # No obstacle, continue driving
                    speed_cap = self.v_nominal
                    bias = 0.0
            
            elif self.state == self.STOP:
                # Stop for 1 second
                speed_cap = 0.0
                bias = 0.0
                
                elapsed = now - self.t_enter
                if elapsed >= self.stop_duration:
                    rospy.loginfo(f"[FSM] Stop complete, transitioning to AVOID")
                    self.trans(self.AVOID)
            
            elif self.state == self.AVOID:
                # Perform avoidance maneuver (semi-circle)
                speed_cap = self.v_avoid
                bias = self.avoid_bias_direction
                
                # Track distance traveled during avoidance
                self.dist_progress += self.v_avoid * dt
                
                # Complete avoidance after traveling avoid_distance
                if self.dist_progress >= self.avoid_distance:
                    rospy.loginfo(f"[FSM] Avoidance complete ({self.dist_progress:.2f}m), returning to DRIVE")
                    self.trans(self.DRIVE)
            
            # Publish outputs
            self.pub_bias.publish(Float32(bias))
            self.pub_speed.publish(Float32(speed_cap))
            rate.sleep()

if __name__ == "__main__":
    node = FSM(node_name='fsm_avoid')
    node.run()
    rospy.spin()
