#!/usr/bin/python
import rospy
from math import pi
from std_msgs.msg import Float32, Int64
from numpy import array


class WheelEncoderPublisher(object):
    """docstring for WheelEncoderPublisher"""

    def __init__(self):
        super(WheelEncoderPublisher, self).__init__()
        rospy.init_node('nadbot_wheel_encoder_publisher')
        # Read in tangential velocity targets
        self.wheel_enc_sub = rospy.Subscriber('wheel_enc', Int64, self.wheel_enc_sub_callback)
        self.wheel_angular_vel_enc_pub = rospy.Publisher('wheel_angular_vel_enc', Float32, queue_size=10)
        self.wheel_tangent_vel_enc_pub = rospy.Publisher('wheel_tangent_vel_enc', Float32, queue_size=10)

        self.rate = rospy.get_param('~rate', 40)
        self.rate_gear = rospy.get_param('~rate_gear', 55)
        self.timeout = float(rospy.get_param('~timeout', 1))
        self.time_prev_update = rospy.Time.now()

        self.R = rospy.get_param('~wheel_radius', 0.0475)
        self.res_encoder = rospy.get_param('~res_encoder', 7)

        # Need a little hack to incorporate direction wheels are spinning
        self.wheel_dir = 1
        self.wheel_vel = 0

        self.wheel_enc = 0
        self.pre_wheel_enc = 0
        self.wheel_angular_vel_enc = 0
        self.angular_wheel_vel = 0

        self.prev_angular_vel = [0.0] * 5

    #####################################################
    def appendVel(self, val):
        #####################################################
        self.prev_angular_vel.append(val)
        del self.prev_angular_vel[0]

    #####################################################
    def calcRollingVel(self):
        #####################################################
        p = array(self.prev_angular_vel)
        self.angular_wheel_vel = p.mean()

    def calAngularVel(self):
        # if (abs(self.wheel_tangent_vel_enc) > 0.005):
        self.appendVel(self.wheel_angular_vel_enc)
        self.calcRollingVel()

    def wheel_enc_sub_callback(self, msg):
        self.wheel_enc = msg.data

    def angular_to_tangent(self, angular):
        return angular * self.R

    def enc_2_rads(self, enc):
        rads = enc * 2.0 * pi / (self.res_encoder * self.rate_gear)
        return rads

    def update(self):
        # History of past three encoder reading
        time_curr_update = rospy.Time.now()
        dt = (time_curr_update - self.time_prev_update).to_sec()

        # Compute angular velocity in rad/s
        wheel_enc_delta = self.wheel_enc - self.pre_wheel_enc
        self.wheel_angular_vel_enc = self.enc_2_rads(wheel_enc_delta) / dt
        self.calAngularVel()
        self.wheel_tangent_vel_enc = self.angular_to_tangent(self.angular_wheel_vel)

        # Publish data
        self.wheel_angular_vel_enc_pub.publish(self.angular_wheel_vel)

        self.wheel_tangent_vel_enc_pub.publish(self.wheel_tangent_vel_enc)
        self.pre_wheel_enc = self.wheel_enc
        self.time_prev_update = time_curr_update

    def spin(self):
        rospy.loginfo("Start nadbot_wheel_encoder_publisher")
        rate = rospy.Rate(self.rate)
        time_curr_update = rospy.Time.now()
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            time_diff_update = (time_curr_update - self.time_prev_update).to_sec()
            if time_diff_update < self.timeout:
                self.update()
            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stop nadbot_wheel_encoder_publisher")
        self.wheel_angular_vel_enc_pub.publish(0)
        self.wheel_tangent_vel_enc_pub.publish(0)
        rospy.sleep(0.5)


def main():
    encoder_publisher = WheelEncoderPublisher()
    encoder_publisher.spin()


if __name__ == '__main__':
    main()
