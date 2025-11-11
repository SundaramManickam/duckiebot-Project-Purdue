#!/usr/bin/env python3
import os
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from duckietown_msgs.msg import LanePose
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

        self.d = 0.0
        self.phi = 0.0
        self.lateral_bias = 0.0
        self.speed_cap = self.v_nominal

        lane_pose_topic = p("topics/lane_pose", f"/{self._vehicle_name}/lane_filter_node/lane_pose")
        rospy.Subscriber(lane_pose_topic, LanePose, self.pose_cb, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/fsm/lateral_offset", Float32, self.bias_cb, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/fsm/speed_cap", Float32, self.speed_cb, queue_size=1)

        cmd_vel_topic = p("topics/cmd_vel_out", f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd")
        self.pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

    def pose_cb(self, msg):
        self.d = msg.d - self.lateral_bias
        self.phi = msg.phi

    def bias_cb(self, msg):
        self.lateral_bias = msg.data

    def speed_cb(self, msg):
        self.speed_cap = msg.data

    def run(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            omega = -(self.k_phi * self.phi + self.k_d * self.d)
            omega = max(min(omega, self.yawrate_limit), -self.yawrate_limit)

            v = min(self.v_nominal, self.speed_cap)
            tw = Twist()
            tw.linear.x = max(v, 0.0)
            tw.angular.z = omega
            self.pub.publish(tw)
            rate.sleep()

if __name__ == "__main__":
    node = ControllerPP(node_name='controller_pp')
    node.run()
    rospy.spin()
