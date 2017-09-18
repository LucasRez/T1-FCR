#!/usr/bin/env python

import sys
import math
import rospy
import tf

from nav_msgs.msg import Odometry


if __name__ == '__main__':
    goal_publisher = rospy.Publisher('goal', Odometry, queue_size=10)
    rospy.loginfo(sys.argv[0])
    rospy.init_node('goal_controller', anonymous=True)
    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw = math.radians(float(sys.argv[3]))
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        msg = Odometry()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        goal_publisher.publish(msg)
        rospy.loginfo("Message Sent")
    else:
        rospy.loginfo("Missing arguments!")
