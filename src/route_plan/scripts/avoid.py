#!/usr/bin/python3
#coding:utf-8
import rospy
from sensor_msgs import LaserScan

class Avoid:
    def __init__(self):
        rospy.Subscriber("/scan",LaserScan,self.laser_cloud_cb)
    
    def laser_cloud_cb(self,msg):
        print(msg)


if __name__ == '__main__':
    node1 = Avoid()
    node1.MainLoop()