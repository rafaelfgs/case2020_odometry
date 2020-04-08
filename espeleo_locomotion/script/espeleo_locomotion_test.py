#!/usr/bin/env python
import rospy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import dynamic_reconfigure.client

motor_publisher1 = rospy.Publisher('/device1/set_joint_state', JointState, queue_size=10)
motor_publisher2 = rospy.Publisher('/device2/set_joint_state', JointState, queue_size=10)
motor_publisher3 = rospy.Publisher('/device3/set_joint_state', JointState, queue_size=10)
motor_publisher4 = rospy.Publisher('/device4/set_joint_state', JointState, queue_size=10)
motor_publisher5 = rospy.Publisher('/device5/set_joint_state', JointState, queue_size=10)
motor_publisher6 = rospy.Publisher('/device6/set_joint_state', JointState, queue_size=10)

last_time = rospy.Time()

velocity_right = 0
velocity_left  = 0

forward_orientation = True

def dyn_config_callback(cfg):
    global forward_orientation

    if "forward_orientation" in cfg.keys():
        forward_orientation = cfg["forward_orientation"]
        print(cfg)

def callback(data):
    global last_time, velocity_right, velocity_left, forward_orientation


    if forward_orientation:
        v = data.linear.x * -1
    else:
        v = data.linear.x

    w = data.angular.z
    r = 0.15
    l = 0.30

    speed_right = (v/r + w*(l/r)) * 14.5
    speed_left = (v/r - w*(l/r)) * 14.5
    
    velocity_right = -speed_right*100
    velocity_left = speed_left*100
    
    last_time = rospy.Time.now()

# def callbackTest(data):
#     print 'locomotio test'
#     print data
    
def listener():
    global last_time, velocity_right, velocity_left

    rospy.init_node('espeleo_locomotion_wheels', anonymous=True)

    rospy.Subscriber("cmd_vel", Twist, callback)

    #rospy.Subscriber("/device1/set_joint_state", JointState, callbackTest)


    r = rospy.Rate(10)

    while not rospy.is_shutdown():
    
        if ((rospy.get_rostime() - last_time).to_sec() > 2):
            stop_state = JointState()
            stop_state.header.stamp = rospy.Time.now()
            stop_state.velocity = [0]

            motor_publisher1.publish(stop_state)
            motor_publisher2.publish(stop_state)
            motor_publisher3.publish(stop_state)
            motor_publisher4.publish(stop_state)
            motor_publisher5.publish(stop_state)
            motor_publisher6.publish(stop_state)
            
            last_time = rospy.Time.now()
        else:
            state_right = JointState()
            state_right.header.stamp = rospy.Time.now()
            state_right.velocity = [velocity_right]

            state_left = JointState()
            state_left.header.stamp = rospy.Time.now()
            state_left.velocity = [velocity_left]

            motor_publisher1.publish(state_right)
            motor_publisher2.publish(state_right)
            motor_publisher3.publish(state_right)
            motor_publisher4.publish(state_left)
            motor_publisher5.publish(state_left)
            motor_publisher6.publish(state_left)

        r.sleep()

if __name__ == '__main__':
    client = dynamic_reconfigure.client.Client("espeleo", timeout=5, config_callback=dyn_config_callback)
    listener()
