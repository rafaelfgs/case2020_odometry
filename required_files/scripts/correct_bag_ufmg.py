#!/usr/bin/env python

import rospy
import rosbag
import numpy
import sys
import os
#from math import pi
from copy import copy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock


input_file = ["/mnt/WD500/Espeleo/2020_02_UFMG/200213_180444_auditorio_autonomo_laser.bag"]

output_file = "/mnt/WD500/Espeleo/2020_02_UFMG/NEW/200213_180444_auditorio_autonomo_laser.bag"

match_bags = False


def qq_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return x, y, z, w

def q_conjugate(q):
    x, y, z, w = q
    return -x, -y, -z, w

def qv_mult(q1, v1):
    q2 = v1 + (0.0,)
    return qq_mult(qq_mult(q1, q2), q_conjugate(q1))[:-1]


def main_function():
    
    rospy.init_node("bagmerge_node", anonymous=True)
    
    for k in range(len(input_file)):
        sys.stdout.write("\n%s" % input_file[k])
        sys.stdout.flush()
    
    if raw_input("\nAre you sure these are the correct files? (y/n): ") == "y":
        sys.stdout.write("\n%s\n" % output_file)
        sys.stdout.flush()
        if os.path.exists(output_file):
            if raw_input("Output file already exists, do you want to replace it? (y/n): ") == "y":
                os.remove(output_file)
            else:
                sys.exit(0)
    else:
        sys.exit(0)
    
    bag_in = numpy.empty(len(input_file), dtype=object)
    bag_time = numpy.zeros((2,len(input_file)))
    
    for k in range(len(input_file)):
        
        sys.stdout.write("\nOpening bag %d..." % (k+1))
        sys.stdout.flush()
        bag_in[k] = rosbag.Bag(input_file[k])
        sys.stdout.write(" Ok!")
        sys.stdout.flush()
        
        bag_time[1,k] = bag_in[k].get_end_time() - bag_in[k].get_start_time()
        for topic, msg, t in bag_in[k].read_messages():
            bag_time[0,k] = rospy.Time.to_sec(t)
            break
    
    if match_bags:
        bag_start_time = min(bag_time[0])
        bag_end_time = min(bag_time[0]) + max(bag_time[1])
        bag_shift_time = bag_time[0] - min(bag_time[0])
    else:
        bag_start_time = min(bag_time[0])
        bag_end_time = max(bag_time[0] + bag_time[1])
        bag_shift_time = numpy.zeros(len(input_file))
    
    sys.stdout.write("\n\nBag starting: %10.9f\nBag ending:   %10.9f\nBag duration: %3.1fs\n" % (bag_start_time, bag_end_time, (bag_end_time-bag_start_time)))
    for k in range(len(input_file)):
        sys.stdout.write("Bag %d shift:  %3.1f\n" % ((k+1), bag_shift_time[k]))
    sys.stdout.flush()
    sys.stdout.write("\n")
    
    bag_out = rosbag.Bag(output_file, "w")
    
    k = 0
    
    while k < len(input_file):
        
        if "200210_173013" in input_file[k]:
            tf_t265  = (-10.7891979218, -8.670835495, -0.227906405926, 0.00645246729255, 0.000483442592667, 0.979790627956, -0.199921101332)
        else:
            tf_t265  = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        
        for topic, msg, t in bag_in[k].read_messages():
            
            if rospy.is_shutdown():
                k = len(input_file)
                break
            
            if match_bags and hasattr(msg, "header"):
                t = rospy.Time.from_sec(rospy.Time.to_sec(t) - bag_shift_time[k])
                msg.header.stamp = copy(t)
            
            if topic == "/ar_pose_marker":
                bag_out.write(topic, msg, t)
            
            elif topic == "/ar_pose_marker_back":
                bag_out.write(topic, msg, t)
            
            elif topic == "/axis_back/camera_info":
                bag_out.write(topic, msg, t)
            
            elif topic == "/axis_back/image_raw/compressed":
                bag_out.write(topic, msg, t)
            
            elif topic == "/axis_front/camera_info":
                bag_out.write(topic, msg, t)
            
            elif topic == "/axis_front/image_raw/compressed":
                bag_out.write(topic, msg, t)
            
            elif topic == "/cmd_vel":
                bag_out.write(topic, msg, t)
            
            elif topic == "/decawave/tag_pose":
                bag_out.write(topic, msg, t)
                uwb_msg = Odometry()
                uwb_msg.header = msg.header
                uwb_msg.pose.pose.position.x = msg.x
                uwb_msg.pose.pose.position.y = msg.y
                uwb_msg.pose.pose.position.z = msg.z
                uwb_msg.pose.pose.orientation.w = 1.0
                bag_out.write("/decawave/odom", uwb_msg, t)
            
            elif topic == "/device1/get_joint_state":
                bag_out.write(topic, msg, t)
            
            elif topic == "/device2/get_joint_state":
                bag_out.write(topic, msg, t)
            
            elif topic == "/device3/get_joint_state":
                bag_out.write(topic, msg, t)
            
            elif topic == "/device4/get_joint_state":
                bag_out.write(topic, msg, t)
            
            elif topic == "/device5/get_joint_state":
                bag_out.write(topic, msg, t)
            
            elif topic == "/device6/get_joint_state":
                bag_out.write(topic, msg, t)
            
            elif topic == "/imu/data":
                tf_msg = TransformStamped()
                tf_msg.header.stamp = copy(t)
                tf_msg.header.frame_id = copy(msg.header.frame_id)
                tf_msg.child_frame_id = "base_footprint"
                tf_msg.transform.translation.x = 0.0
                tf_msg.transform.translation.y = 0.0
                tf_msg.transform.translation.z = 0.0
                tf_msg.transform.rotation.x = 0.0
                tf_msg.transform.rotation.y = 0.0
                tf_msg.transform.rotation.z = 0.0
                tf_msg.transform.rotation.w = 1.0
                bag_out.write("/tf", TFMessage([tf_msg]), t)
                bag_out.write(topic, msg, t)
            
            elif topic == "/laser_odom_to_init":
                bag_out.write(topic, msg, t)
            
            elif topic == "/integrated_to_init":
                bag_out.write(topic, msg, t)
            
            elif topic == "/os1_cloud_node/points":
                bag_out.write(topic, msg, t)
            
            elif topic == "/robot_odom":
                k = 0.01 + abs(msg.pose.pose.orientation.z)
                msg.pose.covariance = [      k,     0.0,     0.0,     0.0,     0.0,     0.0,
                                           0.0,       k,     0.0,     0.0,     0.0,     0.0,
                                           0.0,     0.0,     1.0,     0.0,     0.0,     0.0,
                                           0.0,     0.0,     0.0,     1.0,     0.0,     0.0,
                                           0.0,     0.0,     0.0,     0.0,     1.0,     0.0,
                                           0.0,     0.0,     0.0,     0.0,     0.0, 100.0*k]
                bag_out.write(topic, msg, t)
            
            elif topic == "/realsense/gyro/sample":
                ang_vel = msg.angular_velocity
            
            elif topic == "/realsense/accel/sample":
                if "ang_vel" in locals():
                    imu_data = msg
                    imu_data.linear_acceleration.x = +copy(msg.linear_acceleration.z)
                    imu_data.linear_acceleration.y = -copy(msg.linear_acceleration.x)
                    imu_data.linear_acceleration.z = -copy(msg.linear_acceleration.y)
                    imu_data.angular_velocity.x = +copy(ang_vel.z)
                    imu_data.angular_velocity.y = -copy(ang_vel.x)
                    imu_data.angular_velocity.z = -copy(ang_vel.y)
                    bag_out.write("/realsense/imu/sample", imu_data, t)
            
            elif topic == "/realsense/odom/sample":
                v_old = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
                q_old = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                v_new = qv_mult(q_conjugate(tf_t265[3:]), (v_old[0]-tf_t265[0],v_old[1]-tf_t265[1],v_old[2]-tf_t265[2]))
                q_new = qq_mult(q_conjugate(tf_t265[3:]), q_old)
                msg.pose.pose.position.x = v_new[0]
                msg.pose.pose.position.y = v_new[1]
                msg.pose.pose.position.z = v_new[2]
                msg.pose.pose.orientation.x = q_new[0]
                msg.pose.pose.orientation.y = q_new[1]
                msg.pose.pose.orientation.z = q_new[2]
                msg.pose.pose.orientation.w = q_new[3]
                bag_out.write(topic, msg, t)
                clk_msg = Clock()
                clk_msg.clock = copy(t)
                bag_out.write("/clock", clk_msg, t)
            
            elif topic == "/tf":
                if msg.transforms[0].child_frame_id == "estimation":
                    uwb_ekf_msg = Pose()
                    uwb_ekf_msg.position = copy(msg.transforms[0].transform.translation)
                    uwb_ekf_msg.orientation = copy(msg.transforms[0].transform.rotation)
                    bag_out.write("/espeleo/pose", uwb_ekf_msg, t)
            
            status_time = 100.0 * (rospy.Time.to_sec(t) - bag_start_time) / (bag_end_time - bag_start_time)
            sys.stdout.write("\rRepublishing messages from bag %d... %3.1f%%" % ((k+1), status_time))
            sys.stdout.flush()
        
        sys.stdout.write("\n")
        sys.stdout.flush()
        
        k += 1
    
    for k in range(len(input_file)):
        bag_in[k].close()
    bag_out.close()
    
    sys.stdout.write("\nFiles Closed\n\n")
    sys.stdout.flush()


if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass