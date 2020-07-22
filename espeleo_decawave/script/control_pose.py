#!/usr/bin/env python
import rospy
from ros_decawave.msg import Tag


def callback(data):
	print("AAA\n")
	


if __name__ == '__main__':
	rospy.init_node('Control_System', anonymous=True)
	rospy.Subscriber("/dwc4b8/tag_pose", Tag, callback)
	rospy.spin()
