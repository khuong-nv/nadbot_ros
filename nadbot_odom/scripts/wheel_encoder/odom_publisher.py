#!/usr/bin/python
import rospy
import math
import numpy
import tf
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist

class OdomPublisher(object):
	"""Odom publisher"""
	def __init__(self):
		super(OdomPublisher, self).__init__()
		rospy.init_node('odom_publisher')
		self.lwheel_angular_vel_enc_sub = rospy.Subscriber('lwheel_tangent_vel_enc', Float32, self.lwheel_tangent_vel_enc_callback)
		self.rwheel_angular_vel_enc_sub = rospy.Subscriber('rwheel_tangent_vel_enc', Float32, self.rwheel_tangent_vel_enc_callback)
		self.cmd_vel_enc_pub = rospy.Publisher('cmd_vel_enc', Twist, queue_size=10)
		self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
		self.R = rospy.get_param('~wheel_radius', 0.0475)
		self.L = rospy.get_param('~wheel_separation_distance', 0.344)
		self.rate = rospy.get_param('~rate', 40)
		self.frame_id = rospy.get_param('~frame_id', 'odom')
		self.child_frame_id = rospy.get_param('~child_frame_id', 'base_footprint')

		self.tf_broadcaster = tf.TransformBroadcaster()
		self.lwheel_tangent_vel_enc = 0
		self.rwheel_tangent_vel_enc = 0
		self.pose = {'x':0, 'y': 0, 'th': 0}
		self.time_prev_update = rospy.Time.now()


	def lwheel_tangent_vel_enc_callback(self, msg):
		self.lwheel_tangent_vel_enc = msg.data

	def rwheel_tangent_vel_enc_callback(self, msg):
		self.rwheel_tangent_vel_enc = msg.data

	def angularvel_2_tangentvel(self,angular_vel):
		tangent_vel = angular_vel * self.R
		return tangent_vel

	def pose_next(self, lwheel_tangent_vel_enc, rwheel_tangent_vel_enc):
		x = self.pose['x']; y = self.pose['y']; th = self.pose['th']
		time_curr_update = rospy.Time.now()
		dt = (time_curr_update - self.time_prev_update).to_sec()
		self.time_prev_update = time_curr_update

		if rwheel_tangent_vel_enc == lwheel_tangent_vel_enc:
			v = (lwheel_tangent_vel_enc + rwheel_tangent_vel_enc) / 2.0
			w = 0
			x = x + v*dt*numpy.cos(th)
			y = y + v*dt*numpy.sin(th)
		else:
			v = (lwheel_tangent_vel_enc + rwheel_tangent_vel_enc) / 2.0
			w = (rwheel_tangent_vel_enc - lwheel_tangent_vel_enc) / self.L
			R = ( self.L / 2.0 ) * (lwheel_tangent_vel_enc + rwheel_tangent_vel_enc) / (rwheel_tangent_vel_enc - lwheel_tangent_vel_enc)

			# Update robot pose
			translation = numpy.matrix([[x - R*numpy.sin(th)], [y + R*numpy.cos(th)], [w*dt]])
			icc_pt = numpy.matrix([[R*numpy.sin(th)],[-R*numpy.cos(th)],[th]])
			rotation = numpy.matrix([[numpy.cos(w*dt), -numpy.sin(w*dt), 0],[numpy.sin(w*dt), numpy.cos(w*dt), 0],[0,0,1]])
			pose_next = rotation * icc_pt + translation

			x = pose_next[0,0]
			y = pose_next[1,0]
			th = pose_next[2,0]
		return {'x':x, 'y':y, 'th':th,'v':v,'w':w}

	def pose_update(self):
		# lwheel_tangent_vel_enc = self.angularvel_2_tangentvel(self.lwheel_angular_vel_enc)
		# rwheel_tangent_vel_enc = self.angularvel_2_tangentvel(self.rwheel_angular_vel_enc)
		pose_next = self.pose_next(self.lwheel_tangent_vel_enc, self.rwheel_tangent_vel_enc)
		cmd_vel_enc = Twist()
		cmd_vel_enc.linear.x = pose_next['v']
		cmd_vel_enc.angular.z = pose_next['w']
		self.cmd_vel_enc_pub.publish(cmd_vel_enc)
		return pose_next

	def pub_odometry(self,pose):
		odom_msg = Odometry()
		odom_msg.header.stamp = self.time_prev_update
		odom_msg.header.frame_id = self.frame_id
		odom_msg.child_frame_id = self.child_frame_id
		odom_msg.pose.pose.position = Point(pose['x'], pose['y'], 0)
		odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose['th']))
		self.odom_pub.publish(odom_msg)

	def pub_tf(self,pose):
		self.tf_broadcaster.sendTransform( \
					(pose['x'], pose['y'], 0), \
					tf.transformations.quaternion_from_euler(0,0,pose['th']), \
					self.time_prev_update, \
					self.child_frame_id, \
					self.frame_id \
					)

	def update(self):
		self.pose = self.pose_update()
		# map the orientation to between -pi,pi
		self.pose['th'] = math.atan2(math.sin(self.pose['th']),math.cos(self.pose['th']))
		self.pub_odometry(self.pose)
		self.pub_tf(self.pose)

	def spin(self):
		rospy.loginfo("Start odom publisher")
		rate = rospy.Rate(self.rate)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update()
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Stop odom publisher")
		rospy.sleep(0.5)

def main():
	odom_publisher = OdomPublisher()
	odom_publisher.spin()

if __name__ == '__main__':
	main()
