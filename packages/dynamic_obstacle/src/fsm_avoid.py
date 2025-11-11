#!/usr/bin/env python3
import rospy, time
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
class FSM:
    DRIVE, YIELD, OVERTAKE, RECOVER = "DRIVE","YIELD","OVERTAKE","RECOVER"

    def __init__(self):
        p = rospy.get_param
        self.slow = float(p("slowdown_dist",0.7))
        self.stop = float(p("stop_dist",0.48))
        self.release = self.stop + float(p("release_margin",0.05))
        self.pass_len = float(p("pass_length_m",0.6))
        self.max_cross = float(p("max_cross_time_s",2.0))
        self.v_nominal = float(p("v_nominal",0.25))
        self.v_cross = float(p("v_cross",0.18))
        self.lat_off = float(p("lateral_offset_m",0.15))

        self.range_m = 10.0
        self.opp_free = False
        self.green = True  # assume green if not wired

        tof_topic = p("topics/tof_range", "/dragon/bottom_tof_driver_node/range")
        rospy.Subscriber(tof_topic, Range, self.tof_cb, queue_size=1)
        rospy.Subscriber("/opp_lane_free", Bool, self.opp_cb, queue_size=1)
        rospy.Subscriber("/traffic_light_state", Bool, self.tl_cb, queue_size=1)

        self.pub_bias = rospy.Publisher("/fsm/lateral_offset", Float32, queue_size=1)
        self.pub_speed= rospy.Publisher("/fsm/speed_cap", Float32, queue_size=1)
        self.pub_state= rospy.Publisher("/fsm/state", String, queue_size=1)

        self.state = self.DRIVE
        self.t_enter = time.time()
        self.dist_progress = 0.0
        self.last_t = time.time()

    def tof_cb(self, msg): 
        # sensor_msgs/Range.range is already in meters
        self.range_m = float(msg.range)
    def opp_cb(self, msg): self.opp_free = bool(msg.data)
    def tl_cb(self, msg):  self.green = bool(msg.data)

    def trans(self, new):
        self.state = new
        self.t_enter = time.time()
        self.dist_progress = 0.0
        self.pub_state.publish(String(self.state))

    def run(self):
        rate = rospy.Rate(30)
        self.pub_state.publish(String(self.state))
        while not rospy.is_shutdown():
            now = time.time()
            dt = now - self.last_t
            self.last_t = now

            # crude distance integration ~= v * dt using our own caps
            vcap = self.v_nominal if self.state in [self.DRIVE, self.RECOVER] else self.v_cross
            self.dist_progress += vcap * dt

            # Default outputs
            bias = 0.0
            speed_cap = self.v_nominal

            if not self.green:
                # gate on red
                self.trans(self.YIELD)

            if self.state == self.DRIVE:
                if self.range_m < self.stop:
                    self.trans(self.YIELD)
                elif self.range_m < self.slow:
                    speed_cap = self.v_cross

            elif self.state == self.YIELD:
                speed_cap = 0.0
                if self.range_m > self.release and self.green:
                    # either clear ahead -> resume
                    self.trans(self.DRIVE)
                elif self.opp_free and self.green:
                    # plan an overtake
                    self.trans(self.OVERTAKE)

            elif self.state == self.OVERTAKE:
                bias = self.lat_off
                speed_cap = self.v_cross
                if (self.range_m > self.release and self.dist_progress >= self.pass_len*0.6) \
                    or (now - self.t_enter) > self.max_cross:
                    self.trans(self.RECOVER)

            elif self.state == self.RECOVER:
                speed_cap = self.v_cross
                # fade bias back to 0 over ~0.3 m of progress
                ratio = max(0.0, 1.0 - (self.dist_progress/0.3))
                bias = self.lat_off*ratio
                if ratio <= 0.0:
                    self.trans(self.DRIVE)

            self.pub_bias.publish(Float32(bias))
            self.pub_speed.publish(Float32(speed_cap))
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("fsm_avoid")
    FSM().run()
