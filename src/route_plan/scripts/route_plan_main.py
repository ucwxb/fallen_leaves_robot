#!/usr/bin/python3
#coding:utf-8
import rospy
import os
from vision.msg import leaf_msg,leaf_detect_msg
from std_msgs.msg import Empty,UInt32,Int32
from PID import PID
from communication_scm.msg import *
import numpy as np
from sensor_msgs.msg import LaserScan
import time
class Point:
    def __init__(self,dis,angle,index):
        self.dis = dis
        self.angle = angle
        self.index = index
class RoutePlanNode:
    def __init__(self):
    
        rospy.init_node('route_plan_node', anonymous = True)          #创建节点
        
        self.rate = rospy.Rate(20)
        self.packagePath = rospy.get_param("/pkg_path/route_plan")
        self.pid = PID()
        self.is_handle = 0
        self.stm_vel = stm_vel_cmd()
        self._stm_vel = stm_vel_cmd()
        self.fan = 0
        self.lock = False
        self.x_avoid_vel = 0
        self.y_avoid_vel = 0
        self.current_mode = 0
        self.sweep_status = 0
        self.min_sweep_time = rospy.get_param("/min_sweep_time")
        self.start_sweep_time = 0

        self.is_ban_avoid = 1

        rospy.Subscriber("/leaf_detect",leaf_detect_msg,self.leaf_detect_cb)
        rospy.Subscriber("/switch_mode",Int32,self.switch_mode_cb)
        self.send_stm32_vel = rospy.Publisher("/send_stm32_vel",stm_vel_cmd,queue_size=1)
        rospy.Subscriber("/manual_send_stm32_vel",stm_vel_cmd,self.manual_send_stm32_vel)
        self.send_stm32_fan = rospy.Publisher("/send_stm32_fan",stm_fan_cmd,queue_size=1)
        self.send_stm32_brush = rospy.Publisher("/send_stm32_brush",stm_brush_cmd,queue_size=1)
        self.send_plc_cmd = rospy.Publisher("/send_plc_cmd",plc_plate_cmd,queue_size=1)



        self.min_dis = rospy.get_param("/avoid/min_dis")
        self.max_dis = rospy.get_param("/avoid/max_dis")
        self.danger_dis = rospy.get_param("/avoid/danger_dis")
        self.min_num = rospy.get_param("/avoid/min_num")
        self.ample = rospy.get_param("/avoid/ample")
        rospy.Subscriber("/scan",LaserScan,self.laser_cloud_cb)
        rospy.Subscriber("/switch_avoid",Empty,self.switch_avoid)
        
    def switch_avoid(self,msg):
        self.is_ban_avoid = 1-self.is_ban_avoid

    def laser_cloud_cb(self,msg):
        if self.lock:
            return
        self.lock = True
        self.x_avoid_vel = 0
        self.y_avoid_vel = 0
        if self.is_ban_avoid:
            return
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
            x_vel -= avg*np.cos(avg_angle)
            y_vel -= avg*np.sin(avg_angle)
        
        while np.fabs(x_vel) >= 1.0 :
            x_vel /= 2.0
        while np.fabs(y_vel) >= 1.0:
            y_vel /= 2.0
        x_vel = x_vel * self.ample
        y_vel = y_vel * self.ample
        self.x_avoid_vel = x_vel
        self.y_avoid_vel = y_vel
        self.lock = False

    def manual_send_stm32_vel(self,msg):
        self._stm_vel.x = msg.x
        self._stm_vel.y = msg.y
        self._stm_vel.yaw = msg.yaw

    def switch_mode_cb(self,msg):
        self.current_mode = msg.data
    
    def ready_to_sweep(self):
        self.send_stm32_brush.publish(stm_brush_cmd(rospy.get_param("/ready_to_sweep/brush")))
        self.send_stm32_fan.publish(stm_fan_cmd(rospy.get_param("/ready_to_sweep/fan")))
        self.send_plc_cmd.publish(plc_plate_cmd(rospy.get_param("/ready_to_sweep/front_plate"),rospy.get_param("/ready_to_sweep/after_plate")))
    
    def reset_from_sweep(self):
        self.send_stm32_brush.publish(stm_brush_cmd(0))
        self.send_stm32_fan.publish(stm_fan_cmd(0))
        self.send_plc_cmd.publish(plc_plate_cmd(-10,100))

    def get_leaf_pos(self,res):
        distance = []
        for each_res in res:
            distance.append(np.square(each_res.x)+np.square(each_res.y))
        index = np.argmin(distance)
        return [res[index].x,res[index].y,0]

    def handle_mode(self):
        self.is_handle = 1
        if self.current_mode == 1:
            self.stm_vel.x += self.x_avoid_vel
            self.stm_vel.y += self.y_avoid_vel
            self.send_stm32_vel.publish(self.stm_vel)
        elif self.current_mode == 0:
            self.stm_vel = stm_vel_cmd()
            self._stm_vel = stm_vel_cmd()
            self.send_stm32_vel.publish(stm_vel_cmd(0,0,0))
            self.send_stm32_fan.publish(stm_fan_cmd(0))
            self.send_stm32_brush.publish(stm_brush_cmd(0))
            self.send_plc_cmd.publish(plc_plate_cmd(100,100))
            self.current_mode = -1
        elif self.current_mode == 2:
            self._stm_vel.x += self.x_avoid_vel
            self._stm_vel.y += self.y_avoid_vel
            self.send_stm32_vel.publish(self._stm_vel)
        self.is_handle = 0

    def leaf_detect_cb(self,msg):
        if msg.isFind == 1:
            
            leafPos = self.get_leaf_pos(msg.res)
            res = self.pid.VelPIDController(leafPos)
            self.stm_vel.x = res[0]
            self.stm_vel.y = res[1]
            self.stm_vel.yaw = res[2]
            
            if self.sweep_status == 0:
                self.sweep_status =1
                self.start_sweep_time = time.time()
                self.ready_to_sweep()


        else:
            self.stm_vel.x = self._stm_vel.x
            self.stm_vel.y = self._stm_vel.y
            self.stm_vel.yaw = self._stm_vel.yaw

            if self.sweep_status == 1:
                if self.start_sweep_time -time.time() > self.min_sweep_time:
                    self.sweep_status =0
                    self.reset_from_sweep()
        
    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.handle_mode()

if __name__ == '__main__':
    node1 = RoutePlanNode()
    node1.MainLoop()
