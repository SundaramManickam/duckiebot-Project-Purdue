#!/usr/bin/env python3
import os
import numpy as np
import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import LanePose, WheelsCmdStamped
from duckietown.dtros import DTROS, NodeType


class ControllerPP(DTROS):
    """
    Proportional-Integral (PI) controller for lane following.
    
    Implements PID control with:
    - Proportional terms: k_phi (heading), k_d (lateral)
    - Integral terms: k_Iphi (heading), k_Id (lateral)
    - Error capping to prevent large corrections
    - Integral bounds to prevent windup
    - Feedforward term for open-loop control
    """
    
    def __init__(self, node_name):
        super(ControllerPP, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )
        self._vehicle_name = os.environ['VEHICLE_NAME']
        p = rospy.get_param
        
        # Proportional gains
        self.k_phi = float(p("k_phi", 1.5))      # Heading error gain
        self.k_d = float(p("k_d", 0.5))          # Lateral error gain
        
        # Integral gains
        self.k_Iphi = float(p("k_Iphi", 0.0))    # Heading integral gain
        self.k_Id = float(p("k_Id", 0.0))        # Lateral integral gain
        
        # Speed parameters
        self.v_nominal = float(p("v_nominal", 0.25))
        self.v_cross = float(p("v_cross", 0.18))
        self.yawrate_limit = float(p("yawrate_limit", 1.8))
        self.control_rate = float(p("control_rate_hz", 30))
        
        # Error thresholds (capping)
        self.d_thres = float(p("d_thres", 0.2615))           # Max lateral error
        self.theta_thres_min = float(p("theta_thres_min", -0.5))  # Min heading error
        self.theta_thres_max = float(p("theta_thres_max", 0.75))  # Max heading error
        self.d_offset = float(p("d_offset", 0.0))            # Goal offset from center
        
        # Integral bounds (anti-windup)
        integral_bounds = p("integral_bounds", {
            "d": {"top": 0.3, "bot": -0.3},
            "phi": {"top": 1.2, "bot": -1.2}
        })
        self.integral_bounds_d = (integral_bounds["d"]["bot"], integral_bounds["d"]["top"])
        self.integral_bounds_phi = (integral_bounds["phi"]["bot"], integral_bounds["phi"]["top"])
        
        # Feedforward term
        self.omega_ff = float(p("omega_ff", 0.0))
        
        # Resolution (for integral scaling, if needed)
        self.d_resolution = float(p("d_resolution", 0.011))
        self.phi_resolution = float(p("phi_resolution", 0.051))
        
        # Verbosity
        self.verbose = int(p("verbose", 0))
        
        # Get robot kinematics parameters
        self.wheel_radius = rospy.get_param(f"/{self._vehicle_name}/kinematics_node/radius")
        self.wheel_baseline = rospy.get_param(f"/{self._vehicle_name}/kinematics_node/baseline")
        
        # State variables
        self.d = 0.0
        self.phi = 0.0
        self.lateral_bias = 0.0
        self.speed_cap = self.v_nominal
        
        # Integral terms (accumulated errors)
        self.integral_d = 0.0
        self.integral_phi = 0.0
        
        # Timing for integral calculation
        self.last_time = None
        
        # Subscribers
        lane_pose_topic = p("topics/lane_pose", f"/{self._vehicle_name}/lane_filter_node/lane_pose")
        rospy.Subscriber(lane_pose_topic, LanePose, self.pose_cb, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/fsm/lateral_offset", Float32, self.bias_cb, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/fsm/speed_cap", Float32, self.speed_cb, queue_size=1)
        
        # Publisher
        cmd_vel_topic = p("topics/cmd_vel_out", f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd")
        self.pub = rospy.Publisher(cmd_vel_topic, WheelsCmdStamped, queue_size=1)
        
        rospy.loginfo("[ControllerPP] Initialized with PID control")

    def pose_cb(self, msg):
        """Callback for lane pose updates"""
        self.d = msg.d - self.lateral_bias
        self.phi = msg.phi

    def bias_cb(self, msg):
        """Callback for lateral bias (from FSM)"""
        self.lateral_bias = msg.data
        # Reset integral when bias changes to prevent windup
        if abs(msg.data) > 0.01:
            self.integral_d = 0.0

    def speed_cb(self, msg):
        """Callback for speed cap (from FSM)"""
        self.speed_cap = msg.data

    def compute_control_action(self, d_err, phi_err, dt):
        """
        Compute control action using PID control.
        
        Args:
            d_err: Lateral error (d - d_offset)
            phi_err: Heading error (phi)
            dt: Time step (seconds)
            
        Returns:
            v: Linear velocity (m/s)
            omega: Angular velocity (rad/s)
        """
        # Cap errors to prevent large corrections
        if np.abs(d_err) > self.d_thres:
            d_err = np.sign(d_err) * self.d_thres
        
        if phi_err > self.theta_thres_max:
            phi_err = self.theta_thres_max
        elif phi_err < self.theta_thres_min:
            phi_err = self.theta_thres_min
        
        # Proportional terms
        P_d = self.k_d * d_err
        P_phi = self.k_phi * phi_err
        
        # Integral terms (only if dt is valid)
        I_d = 0.0
        I_phi = 0.0
        
        if dt is not None and dt > 0:
            # Update integrals
            self.integral_d += d_err * dt
            self.integral_phi += phi_err * dt
            
            # Apply integral bounds (anti-windup)
            self.integral_d = np.clip(self.integral_d, self.integral_bounds_d[0], self.integral_bounds_d[1])
            self.integral_phi = np.clip(self.integral_phi, self.integral_bounds_phi[0], self.integral_bounds_phi[1])
            
            # Compute integral contributions
            I_d = self.k_Id * self.integral_d
            I_phi = self.k_Iphi * self.integral_phi
        
        # Total control output
        omega = P_phi + P_d + I_phi + I_d
        
        # Add feedforward term
        omega += self.omega_ff
        
        # Limit angular velocity
        omega = np.clip(omega, -self.yawrate_limit, self.yawrate_limit)
        
        # Linear velocity (capped by FSM)
        v = min(self.v_nominal, self.speed_cap)
        v = max(v, 0.0)
        
        return v, omega

    def twist_to_wheels(self, v, omega):
        """Convert linear velocity (m/s) and angular velocity (rad/s) to wheel velocities (rad/s)"""
        # Differential drive inverse kinematics
        vel_left = (v - 0.5 * omega * self.wheel_baseline) / self.wheel_radius
        vel_right = (v + 0.5 * omega * self.wheel_baseline) / self.wheel_radius
        return vel_left, vel_right

    def run(self):
        rate = rospy.Rate(self.control_rate)
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            
            # Calculate time step
            dt = None
            if self.last_time is not None:
                dt = current_time - self.last_time
            self.last_time = current_time
            
            # Compute errors
            d_err = self.d - self.d_offset
            phi_err = self.phi
            
            # Compute control action
            v, omega = self.compute_control_action(d_err, phi_err, dt)
            
            # Convert to wheel velocities
            vel_left, vel_right = self.twist_to_wheels(v, omega)
            
            # Logging
            if self.verbose >= 1:
                rospy.loginfo_throttle(
                    0.3,
                    f"[PID] d_err={d_err:.3f}, phi_err={phi_err:.3f}, "
                    f"I_d={self.integral_d:.3f}, I_phi={self.integral_phi:.3f}, "
                    f"v={v:.2f}, omega={omega:.2f}, vl={vel_left:.2f}, vr={vel_right:.2f}"
                )
            elif self.verbose == 0:
                rospy.loginfo_throttle(
                    0.3,
                    f"[PID] d={self.d:.3f}, phi={self.phi:.3f}, "
                    f"bias={self.lateral_bias:.3f}, v={v:.2f}, omega={omega:.2f}, "
                    f"vl={vel_left:.2f}, vr={vel_right:.2f}"
                )
            
            # Publish wheel commands
            msg = WheelsCmdStamped(vel_left=vel_left, vel_right=vel_right)
            self.pub.publish(msg)
            rate.sleep()
    
    def on_shutdown(self):
        """Stop motors on shutdown"""
        msg = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
        self.pub.publish(msg)
        rospy.loginfo("ControllerPP: motors stopped")


if __name__ == "__main__":
    node = ControllerPP(node_name='controller_pp')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
