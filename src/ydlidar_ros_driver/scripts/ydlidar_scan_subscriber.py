#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# 该例程将订阅/turtle1/pose话题，消息类型turtlesim::Pose

import rospy
from sensor_msgs.msg import LaserScan
i=0

def poseCallback(msg):
    global i
    if i==0:
     	i=i+1
    	rospy.loginfo(msg)

def pose_subscriber():
	# ROS节点初始化
    rospy.init_node('lidar_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    rospy.Subscriber("scan", LaserScan, poseCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    pose_subscriber()


