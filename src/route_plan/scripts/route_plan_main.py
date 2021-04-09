#!/usr/bin/python3
#coding:utf-8
import rospy
import os
from vision.msg import leaf_msg,leaf_detect_msg
from std_msgs.msg import Empty,UInt32
from PID import PID
from communication_scm.msg import *
import numpy as np

class RoutePlanNode:
    def __init__(self):
    
        rospy.init_node('route_plan_node', anonymous = True)          #创建节点
        
        self.rate = rospy.Rate(20)
        self.packagePath = rospy.get_param("/pkg_path/route_plan")
        self.pid = PID()
        rospy.Subscriber("/leaf_detect",leaf_detect_msg,self.leaf_detect_cb)
        self.current_mode = 0
        rospy.Subscriber("/switch_mode",UInt32,self.switch_mode_cb)
        self.send_stm32_vel = rospy.Publisher("/send_stm32_vel",stm_vel_cmd,queue_size=1)
        self.is_handle = 0
        self.stm_vel = stm_vel_cmd()

        self.brush = 0
        self.send_brush_topic = rospy.Publisher('/send_stm32_brush', stm_brush_cmd,queue_size=1)
        self.fan = 0
        self.send_fan_topic = rospy.Publisher('/send_stm32_fan', stm_fan_cmd,queue_size=1)

    
    def switch_mode_cb(self,msg):
        self.current_mode = msg.data

    def get_leaf_pos(self,res):
        distance = []
        for each_res in res:
            distance.append(np.square(each_res.x)+np.square(each_res.y))
        index = np.argmin(distance)
        return [res[index].x,res[index].y,0]

    def handle_mode(self):
        self.is_handle = 1
        if self.current_mode == 1:
            self.send_stm32_vel.publish(self.stm_vel)
        elif self.current_mode == 0:
            stop_vel = stm_vel_cmd()
            stop_vel.x = 0
            stop_vel.y = 0
            stop_vel.yaw = 0
            self.send_stm32_vel.publish(stop_vel)

        self.is_handle = 0
        
    def leaf_detect_cb(self,msg):
        if msg.isFind == 1:
            
            leafPos = self.get_leaf_pos(msg.res)
            res = self.pid.VelPIDController(leafPos)
            self.stm_vel.x = res[0]
            self.stm_vel.y = res[1]
            self.stm_vel.yaw = res[2]
            
            
        else:
            self.stm_vel.x = 0
            self.stm_vel.y = 0
            self.stm_vel.yaw = 0
        
    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.handle_mode()

if __name__ == '__main__':
    node1 = RoutePlanNode()
    node1.MainLoop()
