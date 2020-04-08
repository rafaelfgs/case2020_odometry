#!/usr/bin/env python

"""
This node receives the motors telemetry from the espeleorobo and calculates the odometry based on the wheels velocity
It subscribes to the wheels velocities in ros_eposmcd/motor1 to motor6, published by ros_eposmcd
It publishes the odometry to odom topic
It can calculate the odometry using differential or skidsteering kinematic models, 
just change the flag skid_steer to 1 if you want skid steer or to 0 if you want differential
The parameters used both for robot and skidsteer come from Eduardo Cota master thesis
https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
"""

from math import sin, cos, pi

import rospy
import genpy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState


class odometry:
    def __init__(self):

        # Kinematic model
        self.skid_steer = 1

        # robot parameters, from cota thesis
        self.wheel_diameter = 0.1475*2
        self.wheel_radius = self.wheel_diameter / 2
        self.robot_width = 0.44
        self.robot_width_internal = 0.35
        self.robot_width_external = 0.48

        # Skidsteer parameters, from cota thesis
        self.alpha_skid = 0.9838227539528335
        self.ycir_skid = 0.3045030420948333 

        # Robot Pose
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.Dth = 0.0

        # Motor velocities
        self.motor_velocity1 = 0
        self.motor_velocity2 = 0
        self.motor_velocity3 = 0
        self.motor_velocity4 = 0
        self.motor_velocity5 = 0
        self.motor_velocity6 = 0


        self.reduction_planetary = 111
        self.reduction_synchronizer = 50.0/26.0
        self.ros_init()



    def ros_init(self):

        # Initialize node
        rospy.init_node('odom_node', anonymous=True)

        # Times used to integrate velocity to pose
        self.current_time = 0.0
        self.last_time = 0.0

        # Create subscribers that receives the wheels velocities
        self.subscriber_motor1 = rospy.Subscriber("/device1/get_joint_state", JointState, self.motor1_callback)
        self.subscriber_motor2 = rospy.Subscriber("/device2/get_joint_state", JointState, self.motor2_callback)
        self.subscriber_motor3 = rospy.Subscriber("/device3/get_joint_state", JointState, self.motor3_callback)
        self.subscriber_motor4 = rospy.Subscriber("/device4/get_joint_state", JointState, self.motor4_callback)
        self.subscriber_motor5 = rospy.Subscriber("/device5/get_joint_state", JointState, self.motor5_callback)
        self.subscriber_motor6 = rospy.Subscriber("/device6/get_joint_state", JointState, self.motor6_callback)

        # odom publisher
        self.odom_pub = rospy.Publisher("/robot_odom", Odometry, queue_size=50)
        self.vel_pub = rospy.Publisher("/robot_vel", Twist, queue_size=1)
        
        # Tf broadcaster
        self.odom_broadcaster = tf.TransformBroadcaster()

    
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        

    # Motor velocity callbacks
    def motor1_callback(self, message):
        self.motor_velocity1 = message.velocity[0]

    def motor2_callback(self, message):
        self.motor_velocity2 = message.velocity[0]

    def motor3_callback(self, message):
        self.motor_velocity3 = message.velocity[0]

    def motor4_callback(self, message):
        self.motor_velocity4 = message.velocity[0]

    def motor5_callback(self, message):
        self.motor_velocity5 = message.velocity[0]

    def motor6_callback(self, message):
        self.motor_velocity6 = message.velocity[0]
        self.current_time = message.header.stamp.secs + message.header.stamp.nsecs*0.000000001
        
        global diff_time
        diff_time = rospy.Time.to_sec(rospy.Time.now()) - genpy.Time.to_sec(message.header.stamp)

        # This is the last motor to have its telemetry sent, so, we use it to call the odometry calculation
        self.odometry_calculation()
        

    def odometry_calculation(self):

        # velocities of each side of the robot, the average of the wheels velocities in RPM
        velocity_left_rpm = ((self.motor_velocity1 + self.motor_velocity2 + self.motor_velocity3))/(self.reduction_planetary * self.reduction_synchronizer * 3)
        # print "Left", velocity_left_rpm
        velocity_right_rpm = ((self.motor_velocity4 + self.motor_velocity5 + self.motor_velocity6))/(self.reduction_planetary * self.reduction_synchronizer * 3)
        # print "Right", velocity_right_rpm


        # Changed RPM to m/s constant value from  0.10471675688 to 0.10471975511965977 -> (2*pi)/60
        # RPM to m/s, and multiplying by the wheel_radius -> (2*pi*radius)/60
        velocity_right = - (0.10471975511965977) * velocity_right_rpm * self.wheel_radius   
        velocity_left = (0.10471975511965977) * velocity_left_rpm * self.wheel_radius
        
    
        if self.skid_steer:
            ### Skid-Steering model

            # Linear velocity
            v_robot = (velocity_right + velocity_left)*(self.alpha_skid/2)

            # Angular velocity
            w_robot = ((velocity_right - velocity_left)*(self.alpha_skid/(2*self.ycir_skid)))

            # Velocity in the XY plane
            x_robot = v_robot * cos(self.th)
            y_robot = v_robot * sin(self.th)
            

            # Calculating odometry
            dt = self.current_time - self.last_time
            delta_x = x_robot * dt
            delta_y = y_robot * dt
            delta_th = w_robot * dt

            # Integrating pose
            self.x += delta_x
            self.y += delta_y
            self.th += delta_th
            

        else:

            # Odometry by dead reckoning

            L = self.robot_width_external/2.0
          
            # v_robot = (velocity_right + velocity_left) / 2
            # w_robot = (velocity_right - velocity_left) / L # * dt # dt???????/ 
        
            # Linear velocity
            v_robot = (velocity_right + velocity_left) / 2.0
            # Angular velocity
            w_robot = (velocity_right - velocity_left) / L

            # Velocity in the XY plane
            x_robot = v_robot * cos(self.th)
            y_robot = v_robot * sin(self.th)
            

            # Calculating odometry
            dt = self.current_time - self.last_time
            delta_x = x_robot * dt
            delta_y = y_robot * dt
            delta_th = w_robot * dt

            # Integrating pose
            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            

            # print "[", self.x, ",", self.y, ",", self.th ,"]"

        

        #Ds = ((velocity_right + velocity_left / 2) * dt) * (self.wheel_radius / 2)
        #Dth = ((velocity_right - velocity_left / 2 * L) * dt) * (self.wheel_radius / 2)
        '''
        if (velocity_right == velocity_left) :
        
                self.th = self.th
                self.x += x_robot * dt * cos(self.th)
                self.y += y_robot * dt * sin(self.th)
            elif (velocity_right == -velocity_left):
        
                self.th += w_robot * dt
                self.x = self.x
                self.y = self.y
            else:
        
                r_center = v_robot / w_robot
                delta_th = (w_robot * dt)
        #self.x = self.x                
        self.x += r_center * (sin(self.th + delta_th) - sin(self.th))
                self.y += - (r_center * (cos(self.th + delta_th) - cos(self.th)))
                
        self.th += delta_th 
                #self.x += Ds*cos((self.th))
        #self.y += Ds*sin((self.th))
        #self.th += Dth
        '''

        vel_robot = Twist()    
        vel_robot.linear.x = v_robot
        vel_robot.angular.z = w_robot


        # Since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # First, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            rospy.Time.from_sec(rospy.Time.to_sec(rospy.Time.now()) - diff_time),
            "odom_link",
            "odom" # odom
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = rospy.Time.from_sec(rospy.Time.to_sec(rospy.Time.now()) - diff_time)
#        odom.header.stamp = self.imu_time
        odom.header.frame_id = "odom" # odom

#	rospy.loginfo(self.x)
        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "odom_link"
        odom.twist.twist = Twist(Vector3(x_robot, y_robot, 0), Vector3(0, 0, w_robot))
        
        k = 0.01 + abs(odom.twist.twist.angular.z)
        odom.pose.covariance = [      k,     0.0,     0.0,     0.0,     0.0,     0.0,
                                    0.0,       k,     0.0,     0.0,     0.0,     0.0,
                                    0.0,     0.0,     1.0,     0.0,     0.0,     0.0,
                                    0.0,     0.0,     0.0,     1.0,     0.0,     0.0,
                                    0.0,     0.0,     0.0,     0.0,     1.0,     0.0,
                                    0.0,     0.0,     0.0,     0.0,     0.0, 100.0*k]
        

#        print w_robot
#        # Calculating the covariance based on the angular velocity
#        # if the robot is rotating, the covariance is higher than if its going straight
#        if abs(w_robot)>0.2:
#            covariance_cons = 0.3
#        else:
#            covariance_cons = 0.05
#
#        odom.pose.covariance[0] = covariance_cons
#        odom.pose.covariance[7] = covariance_cons
#        odom.pose.covariance[35] = 100*covariance_cons
#
#        odom.pose.covariance[1] = covariance_cons
#        odom.pose.covariance[6] = covariance_cons
#
#        odom.pose.covariance[31] = 10*covariance_cons
#        odom.pose.covariance[11] = 10*covariance_cons
#
#        odom.pose.covariance[30] = 10*covariance_cons
#        odom.pose.covariance[5] = 10*covariance_cons
#
#        odom.pose.covariance[14] = 0.1
#        odom.pose.covariance[21] = 0.1
#        odom.pose.covariance[28] = 0.1
#
#        odom.twist.covariance[0] = covariance_cons
#        odom.twist.covariance[7] = covariance_cons
#        odom.twist.covariance[35] = 100*covariance_cons
#
#        odom.twist.covariance[1] = covariance_cons
#        odom.twist.covariance[6] = covariance_cons
#
#        odom.twist.covariance[31] = 10*covariance_cons
#        odom.twist.covariance[11] = 10*covariance_cons
#
#        odom.twist.covariance[30] = 10*covariance_cons
#        odom.twist.covariance[5] = 10*covariance_cons
#
#        odom.twist.covariance[14] = 0.1
#        odom.twist.covariance[21] = 0.1
#        odom.twist.covariance[28] = 0.1



        # publish the message
        self.odom_pub.publish(odom)
        self.vel_pub.publish(vel_robot)

        self.last_time = self.current_time

if __name__ == '__main__':
    odometry_obj = odometry()
