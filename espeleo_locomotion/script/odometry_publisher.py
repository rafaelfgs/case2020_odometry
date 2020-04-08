#!/usr/bin/env python

"""
This node receives the motors telemetry from the espeleorobo and calculates the odometry based on the wheels velocity

It subscribes to the wheels velocities in ros_eposmcd/motor1 to motor6, published by ros_eposmcd
It publishes the odometry to odom topic

It can calculate the odometry using differential or skidsteering kinematic models, 
just change the flag skid_steer to 1 if you want skid steer or to 0 if you want differential

The parameters used both for robot and skidsteer come from Eduardo Cota master thesis
"""

from ros_eposmcd_msgs.msg import EspeleoStatus

from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class odometry:
    def __init__(self):

        # Kinematic model
        self.skid_steer = 1

        # robot parameters, from cota thesis
        self.wheel_diameter = 0.29
        self.wheel_radius = self.wheel_diameter / 2
        self.robot_width = 0.43*2

        # Skidsteer parameters, from cota thesis
        self.alpha_skid = 0.9838227539528335
        self.ycir_skid = 0.3045030420948333 

        # Robot Pose
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Motor velocities
        self.motor_velocity1 = 0
        self.motor_velocity2 = 0
        self.motor_velocity3 = 0
        self.motor_velocity4 = 0
        self.motor_velocity5 = 0
        self.motor_velocity6 = 0

        self.ros_init()

    def ros_init(self):

        # Initialize node
        rospy.init_node('odometry_publisher', anonymous=True)

        # Times used to integrate velocity to pose
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        # Create subscribers that receives the wheels velocities
        self.subscriber_motor1 = rospy.Subscriber("/ros_eposmcd/motor1", EspeleoStatus, self.motor1_callback)
        self.subscriber_motor2 = rospy.Subscriber("/ros_eposmcd/motor2", EspeleoStatus, self.motor2_callback)
        self.subscriber_motor3 = rospy.Subscriber("/ros_eposmcd/motor3", EspeleoStatus, self.motor3_callback)
        self.subscriber_motor4 = rospy.Subscriber("/ros_eposmcd/motor4", EspeleoStatus, self.motor4_callback)
        self.subscriber_motor5 = rospy.Subscriber("/ros_eposmcd/motor5", EspeleoStatus, self.motor5_callback)
        self.subscriber_motor6 = rospy.Subscriber("/ros_eposmcd/motor6", EspeleoStatus, self.motor6_callback)

        # odom publisher
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

        # Tf broadcaster
        self.odom_broadcaster = tf.TransformBroadcaster()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    # Motor velocity callbacks
    def motor1_callback(self, message):
        self.motor_velocity1 = message.speed

    def motor2_callback(self, message):
        self.motor_velocity2 = message.speed

    def motor3_callback(self, message):
        self.motor_velocity3 = message.speed  

    def motor4_callback(self, message):
        self.motor_velocity4 = message.speed

    def motor5_callback(self, message):
        self.motor_velocity5 = message.speed

    def motor6_callback(self, message):
        self.motor_velocity6 = message.speed

        # This is the last motor to have its telemetry sent, so, we use it to call the odometry calculation
        self.odometry_calculation()

    def odometry_calculation(self):

        self.current_time = rospy.Time.now()

        # velocities of each side of the robot, the average of the wheels velocities in RPM
        velocity_right_rpm = (self.motor_velocity1 + self.motor_velocity2 + self.motor_velocity3)/3
        velocity_left_rpm = (self.motor_velocity4 + self.motor_velocity5 + self.motor_velocity6)/3

        # RPM to rad/s, and multiplying by the wheel_radius
        velocity_right = (0.10471675688) * velocity_right_rpm * self.wheel_radius
        velocity_left = -(0.10471675688) * velocity_left_rpm * self.wheel_radius
        
        if self.skid_steer:
            ### Skid-Steering model

            # Linear velocity
            v_robot = (velocity_right + velocity_left)*(self.alpha_skid/2)

            # Angular velocity
            w_robot = (velocity_right - velocity_left)*(self.alpha_skid/(2*self.ycir_skid))
        else:  
            ### Differential model

            # Linear velocity
            v_robot = (velocity_right + velocity_left)/2

            # Angular velocity
            w_robot = (velocity_right - velocity_left)/self.robot_width 

        # Velocity in the XY plane
        x_robot = v_robot * cos(self.th)
        y_robot = v_robot * sin(self.th)

        # Calculating odometry
        dt = (self.current_time - self.last_time).to_sec()
        delta_x = (x_robot * cos(self.th) - y_robot * sin(self.th)) * dt
        delta_y = (x_robot * sin(self.th) + y_robot * cos(self.th)) * dt
        delta_th = w_robot * dt

        # Integrating pose
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # First, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(x_robot, y_robot, 0), Vector3(0, 0, w_robot))

        # Calculating the covariance based on the angular velocity
        # if the robot is rotating, the covariance is higher than if its going straight
        if w_robot>abs(0.2):
            covariance_cons = 0.3
        else:
            covariance_cons = 0.05

        odom.pose.covariance[0] = covariance_cons
        odom.pose.covariance[7] = covariance_cons
        odom.pose.covariance[35] = covariance_cons

        odom.pose.covariance[1] = covariance_cons
        odom.pose.covariance[6] = covariance_cons

        odom.pose.covariance[31] = covariance_cons
        odom.pose.covariance[11] = covariance_cons

        odom.pose.covariance[30] = covariance_cons
        odom.pose.covariance[5] = covariance_cons

        odom.pose.covariance[14] = 0.1
        odom.pose.covariance[21] = 0.1
        odom.pose.covariance[28] = 0.1

        odom.twist.covariance[0] = covariance_cons
        odom.twist.covariance[7] = covariance_cons
        odom.twist.covariance[35] = covariance_cons

        odom.twist.covariance[1] = covariance_cons
        odom.twist.covariance[6] = covariance_cons

        odom.twist.covariance[31] = covariance_cons
        odom.twist.covariance[11] = covariance_cons

        odom.twist.covariance[30] = covariance_cons
        odom.twist.covariance[5] = covariance_cons

        odom.twist.covariance[14] = 0.1
        odom.twist.covariance[21] = 0.1
        odom.twist.covariance[28] = 0.1



        # publish the message
        self.odom_pub.publish(odom)

        self.last_time = self.current_time

if __name__ == '__main__':
    odometry_obj = odometry()