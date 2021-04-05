#!/usr/bin/python3
#coding:utf-8
import rospy
import os
from vision.msg import leaf_msg,leaf_detect_msg
from PID import PID
from communication_scm.msg import stm_vel_cmd
class RoutePlanNode:
    def __init__(self):
    
        rospy.init_node('route_plan_node', anonymous = True)          #创建节点
        
        self.rate = rospy.Rate(20)
        self.packagePath = rospy.get_param("/pkg_path/route_plan")
        self.pid = PID()
        rospy.Subscriber("/leaf_detect",leaf_detect_msg,self.leaf_detect_cb)
        self.send_stm32_vel = rospy.Publisher("/send_stm32_vel",stm_vel_cmd,queue_size=1)
        self.is_handle = 0
    
    def leaf_detect_cb(self,msg):
        if self.is_handle == 0 and msg.isFind == 1:
            self.is_handle = 1
            one_leaf_info = msg.res[0]
            leafPos = [one_leaf_info.x,one_leaf_info.y,0]
            stm_vel = stm_vel_cmd()
            res = self.pid.VelPIDController(leafPos)
            stm_vel.x = res[0]
            stm_vel.y = res[1]
            stm_vel.yaw = res[2]
            self.send_stm32_vel.publish(stm_vel)
            self.is_handle = 0
        
    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    node1 = RoutePlanNode()
    node1.MainLoop()