#!/usr/bin/python3
#coding:utf-8
import serial
import Jetson.GPIO
import time
import threading
import rospy
from std_msgs.msg import String,Int32
from communication_scm.msg import *
import numpy as np
import struct
class Com:
    def __init__(self):
        rospy.init_node('communication_node', anonymous = True)
        self.pkg_path = rospy.get_param("/pkg_path/communication_scm")
        self.serial_path = rospy.get_param("/serial_path")
        self.plc_serial_path = rospy.get_param("/plc_serial_path")
        self.serial_rate = int(rospy.get_param("/serial_rate"))

        self.rate = rospy.Rate(20)

        rospy.Subscriber("/send_stm32",String,self.send_stm32_cb)
        self.receive_stm32 = rospy.Publisher("/receive_stm32", String,queue_size=1)

        rospy.Subscriber("/send_plc",String,self.send_plc_cb)
        self.receive_plc = rospy.Publisher("/receive_plc", String,queue_size=1)

        rospy.Subscriber("/send_stm32_vel",stm_vel_cmd,self.send_stm32_vel)
        rospy.Subscriber("/send_stm32_fan",stm_fan_cmd,self.send_stm32_fan)
        rospy.Subscriber("/send_stm32_brush",stm_brush_cmd,self.send_stm32_brush)
        rospy.Subscriber("/send_plc_cmd",plc_cmd,self.send_plc_cmd)

        self.init_serial()
        # self.init_threading()
        
        self.max_vel = rospy.get_param("/max_vel")
        self.min_vel = rospy.get_param("/min_vel")
        self.max_ang_vel = rospy.get_param("/max_ang_vel")
        self.min_ang_vel = rospy.get_param("/min_ang_vel")
    
    def init_serial(self):
        times = 0
        while(1):
            try:
                if times >= 1:
                    print("failed to connect to stm32")
                    self.ser = None
                    break
                self.ser = serial.Serial(self.serial_path, self.serial_rate,timeout=1)
                print("success connect to stm32")
                break
            except:
                times += 1
                print("stm32 serial time out")
            time.sleep(1)
        times = 0
        while(1):
            try:
                if times >= 2:
                    print("failed to connect to plc")
                    self.plc_ser = None
                    break
                self.plc_ser = serial.Serial(self.plc_serial_path, self.serial_rate,timeout=1)
                print("success connect to plc")
                break
            except:
                times += 1
                print("plc serial time out")
            time.sleep(1)
    
    def init_threading(self):
        self.t = threading.Thread(target=self.receive_stm32_func)
        # self.t.start()
        self.plc_t = threading.Thread(target=self.receive_plc_func)
        # self.plc_t.start()

    def send_stm32_cb(self,msg):
        data = msg.data
        self.send_stm32(data)

    def send_stm32(self,string):
        self.ser.write(string)

    def receive_stm32_func(self):
        if self.ser != None and self.ser.isOpen():
            res = self.ser.readall()
            res = bytes.decode(res)
            if res != '':
                self.receive_stm32.publish(res)
                print(res)
    
    def filter(self,src,max_num,min_num):
        if np.fabs(src) >= np.fabs(max_num):
            if src < 0:
                return -max_num
            return max_num
        if np.fabs(src) <= np.fabs(min_num):
            if src < 0:
                return -min_num
            return min_num
        return src

    def send_stm32_vel(self,msg):
        vel_x = msg.x
        vel_y = msg.y
        vel_yaw = msg.yaw
        msg_type = 1
        if vel_x != 0:
            vel_x = self.filter(vel_x,self.max_vel,self.min_vel)
        if vel_y != 0:
            vel_y = self.filter(vel_y,self.max_vel,self.min_vel)
        if vel_yaw != 0:
            vel_yaw = self.filter(vel_yaw,self.max_ang_vel,self.min_ang_vel)
        print(vel_x,vel_y,vel_yaw)
        vel_x = struct.pack('f',vel_x)
        
        vel_y = struct.pack('f',vel_y)
        vel_yaw = struct.pack('f',vel_yaw)
        # print(vel_x,vel_y,vel_yaw)
        cmd_string = b''
        cmd_string += bytes([0xFF])
        cmd_string += bytes([msg_type])
        for i in vel_x:
            cmd_string += bytes([i])
        for i in vel_y:
            cmd_string += bytes([i])
        for i in vel_yaw:
            cmd_string += bytes([i])
        # print(cmd_string)
        self.send_stm32(cmd_string)
    
    def send_stm32_fan(self,msg):
        vel = msg.vel
        msg_type = 3
        vel = struct.pack('f',vel)
        cmd_string = b''
        cmd_string += bytes([0xFF])
        cmd_string += bytes([msg_type])
        for i in vel:
            cmd_string += bytes([i])
        # print(cmd_string)
        self.send_stm32(cmd_string)
    
    def send_stm32_brush(self,msg):
        vel = msg.vel
        msg_type = 2
        vel = struct.pack('f',vel)
        cmd_string = b''
        cmd_string += bytes([0xFF])
        cmd_string += bytes([msg_type])
        for i in vel:
            cmd_string += bytes([i])
        # print(cmd_string)
        self.send_stm32(cmd_string)

    def send_plc_cb(self,msg):
        data = msg.data
        self.send_plc(data)
    
    def send_plc(self,string):
        self.plc_ser.write(string)
    
    def receive_plc_func(self):
        # while(1):
        if self.plc_ser != None and self.plc_ser.isOpen():
            res = self.plc_ser.readall()
            res = bytes.decode(res)
            if res != '':
                self.receive_plc.publish(res)
                print(res)

    def send_plc_cmd(self,msg):
        cmd_slisde_dis = msg.slide_dis
        cmd_arm_dis = msg.arm_dis
        if np.fabs(cmd_slisde_dis)>250:
            cmd_slisde_dis = 0
        if np.fabs(cmd_arm_dis)>250:
            cmd_arm_dis = 0
        cmd_string = "%d %d"%(cmd_slisde_dis,cmd_arm_dis)
        cmd_string = bytes(cmd_string,encoding="utf8")
        self.send_plc(cmd_string)


    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.receive_plc_func()
            self.receive_stm32_func()
            

if __name__ == '__main__':
    node1 = Com()
    node1.MainLoop()
