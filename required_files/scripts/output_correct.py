#!/usr/bin/env python
import rospy
import sys
from copy import copy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

sensor_name = sys.argv[1]
input_topic = sys.argv[2]
tf_sensor = tuple([float(x) for x in sys.argv[3].split(' ')])
tf_world = tuple([float(x) for x in sys.argv[4].split(' ')])

odom_seq = 0


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


def callback_clock(data):
    global clock_time
    clock_time = data.clock


def callback(data):
    
    global odom_seq
    
    odom_seq += 1
    pub = rospy.Publisher("/" + sensor_name + "/odom", Odometry, queue_size=2)
    
    odom = Odometry()
    odom.header.seq = odom_seq
    odom.header.frame_id = "world"
    odom.child_frame_id = sensor_name + "_pose"
    
    if sensor_name != "uwb_ekf":
        odom.header.stamp = copy(data.header.stamp)
        odom.pose.covariance = data.pose.covariance
        v_sensor = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
        q_sensor = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
    else:
        odom.header.stamp = clock_time
        v_sensor = (data.position.x, data.position.y, data.position.z)
        q_sensor = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    
    v_init = qv_mult(q_sensor, (-tf_sensor[0], -tf_sensor[1], -tf_sensor[2]))
    v_init = (v_sensor[0] + v_init[0], v_sensor[1] + v_init[1], v_sensor[2] + v_init[2])
    q_init = qq_mult(q_conjugate(tf_sensor[3:]), q_sensor)
    
    v_base = qv_mult(tf_sensor[3:], (v_init[0]+tf_sensor[0], v_init[1]+tf_sensor[1], v_init[2]+tf_sensor[2]))
    q_base = qq_mult(tf_sensor[3:], q_init)
    
    if not "uwb" in sensor_name:
        v_world = qv_mult(tf_world[3:], (v_base[0]+tf_world[0], v_base[1]+tf_world[1], v_base[2]+tf_world[2]))
        q_world = qq_mult(tf_world[3:], q_base)
    else:
        v_world = v_base
        q_world = q_base
        
    if sensor_name == "lidar_noimu":
        q_world = (q_world[0], q_world[2], q_world[1], q_world[3])
    
    odom.pose.pose.position.x = v_world[0]
    odom.pose.pose.position.y = v_world[1]
    odom.pose.pose.position.z = v_world[2]
    
    odom.pose.pose.orientation.x = q_world[0]
    odom.pose.pose.orientation.y = q_world[1]
    odom.pose.pose.orientation.z = q_world[2]
    odom.pose.pose.orientation.w = q_world[3]
    
    pub.publish(odom)


def main_function():

    rospy.init_node("output_" + sensor_name + "_node", anonymous=True)
    rospy.Subscriber("/clock", Clock, callback_clock)
    
    while not "clock_time" in globals():
        pass
    
    if sensor_name == "wheel_ekf":
        rospy.Subscriber(input_topic, PoseWithCovarianceStamped, callback)
    if sensor_name == "uwb_ekf":
        rospy.Subscriber(input_topic, Pose, callback)
    else:
        rospy.Subscriber(input_topic, Odometry, callback)

    sys.stdout.write("Republishing odom from %s in respect to world\n" % sensor_name)
    sys.stdout.flush()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass