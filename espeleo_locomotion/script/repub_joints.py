#!/usr/bin/env python

import rospy
import sys
from espeleo_decawave.msg import Tag
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class repub:
	def __init__(self):
		rospy.init_node('repub_joints', anonymous=True)
                rospy.Subscriber('/imu/data', Imu, self.callback_imu)
                while not 'bag_time' in globals(): pass
		self.motor_publisher1 = rospy.Publisher('/device1/get_joint_state', JointState, queue_size=1)
		self.motor_publisher2 = rospy.Publisher('/device2/get_joint_state', JointState, queue_size=1)
		self.motor_publisher3 = rospy.Publisher('/device3/get_joint_state', JointState, queue_size=1)
		self.motor_publisher4 = rospy.Publisher('/device4/get_joint_state', JointState, queue_size=1)
		self.motor_publisher5 = rospy.Publisher('/device5/get_joint_state', JointState, queue_size=1)
		self.motor_publisher6 = rospy.Publisher('/device6/get_joint_state', JointState, queue_size=1)
		
		self.last_time = rospy.Time()
		self.velocity_right = 0.0
		self.velocity_left = 0.0
		self.forward_orientation = True

		rospy.Subscriber("/cmd_vel", Twist, self.callback_vel)


        def callback_imu(self, data):
                global bag_time
                bag_time = data.header.stamp
                
#                t = TransformStamped()
#                t.header.stamp = bag_time
#                t.header.frame_id = 'wheel_imu_pose'
#                t.child_frame_id = 'imu'
#                t.transform.translation.x = 0.0
#                t.transform.translation.y = 0.0
#                t.transform.translation.z = 0.235
#                t.transform.rotation.x = 0.0
#                t.transform.rotation.y = 0.0
#                t.transform.rotation.z = 0.0
#                t.transform.rotation.w = 1.0
#                imu_tf = rospy.Publisher('/tf', TFMessage, queue_size=1)
#                imu_tf.publish(TFMessage([t]))
                
#                t = TFMessage()
#                t.transforms[0].header.stamp = bag_time
#                t.transforms[0].header.frame_id = 'wheel_imu_pose'
#                t.transforms[0].child_frame_id = 'imu'
#                t.transforms[0].transform.translation.z = 0.235
#                t.transforms[0].transform.rotation.w = 1.0
#                
#                t.transforms[1].header.stamp = bag_time
#                t.transforms[1].header.frame_id = 'chassis_init'
#                t.transforms[1].child_frame_id = 'wheel_init'
#                t.transforms[1].transform.rotation.w = 1.0
#                
#                t.transforms[2].header.stamp = bag_time
#                t.transforms[2].header.frame_id = 'chassis_init'
#                t.transforms[2].child_frame_id = 'wheel_imu_init'
#                t.transforms[2].transform.rotation.w = 1.0
#                
#                t.transforms[3].header.stamp = bag_time
#                t.transforms[3].header.frame_id = 'wheel_pose'
#                t.transforms[3].child_frame_id = 'wheel_chassis'
#                t.transforms[3].transform.rotation.w = 1.0
#                
#                t.transforms[4].header.stamp = bag_time
#                t.transforms[4].header.frame_id = 'wheel_imu_pose'
#                t.transforms[4].child_frame_id = 'wheel_imu_chassis'
#                t.transforms[4].transform.rotation.w = 1.0
                
                ti = TransformStamped()
                ti.header.stamp = bag_time
                ti.header.frame_id = 'wheel_imu_pose'
                ti.child_frame_id = 'imu'
                ti.transform.translation.z = 0.235
                ti.transform.rotation.w = 1.0
                
                twi = TransformStamped()
                twi.header.stamp = bag_time
                twi.header.frame_id = 'chassis_init'
                twi.child_frame_id = 'wheel_init'
                twi.transform.rotation.w = 1.0
                
                twii = TransformStamped()
                twii.header.stamp = bag_time
                twii.header.frame_id = 'chassis_init'
                twii.child_frame_id = 'wheel_imu_init'
                twii.transform.rotation.w = 1.0
                
                twc = TransformStamped()
                twc.header.stamp = bag_time
                twc.header.frame_id = 'wheel_pose'
                twc.child_frame_id = 'wheel_chassis'
                twc.transform.rotation.w = 1.0
                
                twic = TransformStamped()
                twic.header.stamp = bag_time
                twic.header.frame_id = 'wheel_imu_pose'
                twic.child_frame_id = 'wheel_imu_chassis'
                twic.transform.rotation.w = 1.0
                
                t = TFMessage([ti,twi,twii,twc,twic])
                
                pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=1)
                pub_tf.publish(t)
                


	def callback_vel(self, data):
		if self.forward_orientation:
			v = data.linear.x * -1
		else:
			v = data.linear.x

		w = data.angular.z
		r = 0.15
		l = 0.30

		speed_right = (v/r + w*(l/r)) * 14.5
		speed_left = (v/r - w*(l/r)) * 14.5

		self.velocity_right = -speed_right*100
		self.velocity_left = speed_left*100


		self.last_time = rospy.Time.now()


	def run(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			state_right = JointState()
			state_right.header.stamp = bag_time
			state_right.velocity = [self.velocity_right]

			state_left = JointState()
			state_left.header.stamp = bag_time
			state_left.velocity = [self.velocity_left]

			self.motor_publisher1.publish(state_right)
			self.motor_publisher2.publish(state_right)
			self.motor_publisher3.publish(state_right)
			self.motor_publisher4.publish(state_left)
			self.motor_publisher5.publish(state_left)
			self.motor_publisher6.publish(state_left)

			rate.sleep()

if __name__ == '__main__':
	try:
		node = repub()
		node.run()
	except rospy.ROSInterruptException:
		pass
