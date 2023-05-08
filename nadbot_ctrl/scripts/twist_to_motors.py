#!/usr/bin/env python
# Author: khuongnv@nadrobot
import rospy
import roslib
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import Twist


class TwistToMotors():
	def __init__(self):
		rospy.init_node("twist_to_motors")
		nodename = rospy.get_name()
		rospy.loginfo("%s started" % nodename)

		self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32, queue_size=10)
		self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32, queue_size=10)
		rospy.Subscriber('cmd_vel', Twist, self.twistCallback)

		self.w = rospy.get_param("~wheel_separation", 0.34)
		self.rate = rospy.get_param("~rate", 40)
		self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)

		self.left = 0
		self.right = 0
		self.time_prev_update = rospy.Time.now()
		self.dx = 0
		self.dr = 0

	def spin(self):
		r = rospy.Rate(self.rate)
		time_curr_update = rospy.Time.now()
		while not rospy.is_shutdown():
			time_diff_update = (time_curr_update - self.time_prev_update).to_sec()
			if time_diff_update < self.timeout_ticks:
				self.spinOnce()
			r.sleep()
		rospy.spin()

	def spinOnce(self):
		self.right = 1.0 * self.dx + self.dr * self.w / 2.0
		self.left = 1.0 * self.dx - self.dr * self.w / 2.0
		self.pub_lmotor.publish(self.left)
		self.pub_rmotor.publish(self.right)

	def twistCallback(self, msg):
		self.dx = msg.linear.x
		self.dr = msg.angular.z
		self.time_prev_update = rospy.Time.now()


if __name__ == '__main__':
	try:
		twistToMotors = TwistToMotors()
		twistToMotors.spin()
	except rospy.ROSInterruptException:
		pass
