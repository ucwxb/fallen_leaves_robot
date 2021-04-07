#!/usr/bin/python3
#coding:utf-8
import rospy
import os
from vision.msg import leaf_msg,leaf_detect_msg
from std_msgs.msg import Empty
from PID import PID
from communication_scm.msg import stm_vel_cmd
import numpy as np
STOP = 0
RUN = 1
MODE = [STOP,RUN]
class RoutePlanNode:
    def __init__(self):
    
        rospy.init_node('route_plan_node', anonymous = True)          #创建节点
        
        self.rate = rospy.Rate(20)
        self.packagePath = rospy.get_param("/pkg_path/route_plan")
        self.pid = PID()
        rospy.Subscriber("/leaf_detect",leaf_detect_msg,self.leaf_detect_cb)
        self.current_mode = RUN
        rospy.Subscriber("/switch_mode",Empty,self.switch_mode_cb)
        self.send_stm32_vel = rospy.Publisher("/send_stm32_vel",stm_vel_cmd,queue_size=1)
        self.is_handle = 0
    
    def switch_mode_cb(self):
        self.current_mode += 1
        self.current_mode %= len(MODE)

    def get_leaf_pos(self,res):
        distance = []
        for each_res in res:
            distance.append(np.square(each_res.x)+np.square(each_res.y))
        index = np.argmin(distance)
        return [res[index].x,res[index].y,0]

    def leaf_detect_cb(self,msg):
        stm_vel = stm_vel_cmd()
        if self.is_handle == 0 and msg.isFind == 1 and self.current_mode == RUN:
            self.is_handle = 1
            leafPos = self.get_leaf_pos(msg.res)
            res = self.pid.VelPIDController(leafPos)
            stm_vel.x = res[0]
            stm_vel.y = res[1]
            stm_vel.yaw = res[2]
            self.send_stm32_vel.publish(stm_vel)
            self.is_handle = 0
        else:
            stm_vel.x = 0
            stm_vel.y = 0
            stm_vel.yaw = 0
            self.send_stm32_vel.publish(stm_vel)
        
    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    node1 = RoutePlanNode()
    node1.MainLoop()
