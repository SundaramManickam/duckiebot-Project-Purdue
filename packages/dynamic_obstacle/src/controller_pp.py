#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import LanePose, WheelsCmdStamped
from duckietown.dtros import DTROS, NodeType


class ControllerPP(DTROS):
    def __init__(self, node_name):
        super(ControllerPP, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )
        self._vehicle_name = os.environ['VEHICLE_NAME']
        p = rospy.get_param
        self.k_phi = float(p("k_phi", 2.5))
        self.k_d   = float(p("k_d", 2.0))
        self.v_nominal = float(p("v_nominal", 0.25))
        self.v_cross   = float(p("v_cross", 0.18))
        self.yawrate_limit = float(p("yawrate_limit", 1.8))
        self.control_rate = float(p("control_rate_hz", 30))

        # Get robot kinematics parameters
        self.wheel_radius = rospy.get_param(f"/{self._vehicle_name}/kinematics_node/radius")
        self.wheel_baseline = rospy.get_param(f"/{self._vehicle_name}/kinematics_node/baseline")

        self.d = 0.0
        self.phi = 0.0
        self.lateral_bias = 0.0
        self.speed_cap = self.v_nominal

        lane_pose_topic = p("topics/lane_pose", f"/{self._vehicle_name}/lane_filter_node/lane_pose")
        rospy.Subscriber(lane_pose_topic, LanePose, self.pose_cb, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/fsm/lateral_offset", Float32, self.bias_cb, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/fsm/speed_cap", Float32, self.speed_cb, queue_size=1)

        cmd_vel_topic = p("topics/cmd_vel_out", f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd")
        self.pub = rospy.Publisher(cmd_vel_topic, WheelsCmdStamped, queue_size=1)

    def pose_cb(self, msg):
        self.d = msg.d - self.lateral_bias
        self.phi = msg.phi

    def bias_cb(self, msg):
        self.lateral_bias = msg.data

    def speed_cb(self, msg):
        self.speed_cap = msg.data

    def twist_to_wheels(self, v, omega):
        """Convert linear velocity (m/s) and angular velocity (rad/s) to wheel velocities (rad/s)"""
        # Differential drive inverse kinematics
        vel_left = (v - 0.5 * omega * self.wheel_baseline) / self.wheel_radius
        vel_right = (v + 0.5 * omega * self.wheel_baseline) / self.wheel_radius
        return vel_left, vel_right

    def run(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            omega = (self.k_phi * self.phi + self.k_d * self.d) # was -
            omega = max(min(omega, self.yawrate_limit), -self.yawrate_limit)
            # omega = 0.0 #placeholder
            v = min(self.v_nominal, self.speed_cap)
            v = max(v, 0.0)
            # v = 0.0 #placeholder

            # Convert to wheel velocities
            vel_left, vel_right = self.twist_to_wheels(v, omega)
            rospy.loginfo_throttle(
                0.3,
                f"[PP] d={self.d:.3f}, phi={self.phi:.3f}, "
                f"bias={self.lateral_bias:.3f}, v={v:.2f}, "
                f"omega={omega:.2f}, vl={vel_left:.2f}, vr={vel_right:.2f}"
            )

            # Publish wheel commands
            msg = WheelsCmdStamped(vel_left=vel_left, vel_right=vel_right)
            self.pub.publish(msg)
            rate.sleep()
    
    def on_shutdown(self):
        msg=WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
        self.pub.publish(msg)
        rospy.loginfo("ControllerPP: motors stopped")


if __name__ == "__main__":
    #rospy.init_node("controller_pp")
    node = ControllerPP(node_name='controller_pp')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    #rospy.spin()
