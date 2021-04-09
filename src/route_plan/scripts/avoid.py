#!/usr/bin/python3
#coding:utf-8
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
class Point:
    def __init__(self,dis,angle,index):
        self.dis = dis
        self.angle = angle
        self.index = index

class Avoid:
    def __init__(self):
        rospy.init_node('avoid_node', anonymous = True)          #创建节点
    
        self.rate = rospy.Rate(20)
        rospy.Subscriber("/scan",LaserScan,self.laser_cloud_cb)
        self.min_dis = 0.1
        self.max_dis = 6.0
        self.danger_dis = 1.0
        self.min_num = 10
        self.ample = 40
        self.lock = False
    
    def laser_cloud_cb(self,msg):
        if self.lock:
            return
        self.lock = True
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        res_list = []
        flag = 0
        for index,val in enumerate(ranges):
            if val <= self.max_dis and val >= self.min_dis:
                if val <= self.danger_dis:
                    if flag == 0:
                        temp_list = []
                        flag = 1
                    # print("angle",val,np.pi-index*angle_increment)
                    temp_list.append(Point(val,np.pi-index*angle_increment,index))
                else:
                    if flag == 1:
                        flag = 0
                        if len(temp_list) > self.min_num:
                            res_list.append(temp_list)
                        temp_list = []
            else:
                if flag == 1:
                    flag = 0
                    if len(temp_list) > self.min_num:
                        res_list.append(temp_list)
                    temp_list = []
        x_vel = 0
        y_vel = 0
        for each_list in res_list:
            avg = 0
            avg_angle = 0
            for i in each_list:
                avg += i.dis
                avg_angle +=  i.angle
            avg /= len(each_list)
            avg = self.danger_dis - avg
            avg_angle /= len(each_list)
            # print("avg:",avg,"avg_angle:",avg_angle)
            x_vel -= avg*np.cos(avg_angle)
            y_vel -= avg*np.sin(avg_angle)
            # temp_y = self.danger_dis - np.fabs(avg*np.sin(avg_angle))
            # if avg*np.cos(avg_angle) > 0:
            #     x_vel -= temp_x
            # else:
            #     x_vel += temp_x
            
            # if avg*np.sin(avg_angle) > 0:
            #     y_vel -= temp_y
            # else:
            #     y_vel += temp_y
        
        while np.fabs(x_vel) >= 1.0 :
            x_vel /= 2.0
        while np.fabs(y_vel) >= 1.0:
            y_vel /= 2.0
        print("raw",x_vel,y_vel)
        x_vel = x_vel * self.ample
        y_vel = y_vel * self.ample
        print(x_vel,y_vel)
        self.lock = False

    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    node1 = Avoid()
    node1.MainLoop()