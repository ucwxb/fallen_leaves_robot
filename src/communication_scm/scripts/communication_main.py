#!/usr/bin/python3
#coding:utf-8
import serial
# import Jetson.GPIO
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

        self.receive_stm32 = rospy.Publisher("/receive_stm32", String,queue_size=1)
        self.receive_plc = rospy.Publisher("/receive_plc", String,queue_size=1)

        rospy.Subscriber("/send_stm32_vel",stm_vel_cmd,self.send_stm32_vel)
        rospy.Subscriber("/send_stm32_fan",stm_fan_cmd,self.send_stm32_fan)
        rospy.Subscriber("/send_stm32_brush",stm_brush_cmd,self.send_stm32_brush)
        rospy.Subscriber("/send_plc_cmd",plc_plate_cmd,self.send_plc_cmd)

        self.init_serial()

        self.enable_plc_receive = True
        self.enable_stm_receive = False
    
    def init_serial(self):
        times = 0
        while(1):
            try:
                if times >= 2:
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

    def send_stm32(self,string):
        try:
            self.ser.write(string)
        except:
            return

    def receive_stm32_func(self):
        if self.enable_plc_receive == False:
            return 
        if self.ser != None and self.ser.isOpen():
            try:
                res = self.ser.readall()
                res = bytes.decode(res)

                if res != '':
                    self.receive_stm32.publish(res)
            except:
                return

    def send_stm32_vel(self,msg):
        vel_x = msg.x
        vel_y = msg.y
        vel_yaw = msg.yaw
        msg_type = 1

        vel_x = struct.pack('f',vel_x)
        vel_y = struct.pack('f',vel_y)
        vel_yaw = struct.pack('f',vel_yaw)
        cmd_string = b''
        cmd_string += bytes([0xFF])
        cmd_string += bytes([msg_type])
        for i in vel_x:
            cmd_string += bytes([i])
        for i in vel_y:
            cmd_string += bytes([i])
        for i in vel_yaw:
            cmd_string += bytes([i])
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
        self.send_stm32(cmd_string)
    
    def send_plc(self,string):
        try:
            self.plc_ser.write(string)
        except:
            return
    
    def receive_plc_func(self):
        if self.enable_plc_receive == False:
            return 
        if self.plc_ser != None and self.plc_ser.isOpen():
            try:
                res = self.plc_ser.read(1)
                print(res)
                if res == bytes([0xbb]):
                    print("receive_plc")
                if res != '':
                    self.receive_plc.publish(res)
            except:
                print("failed to read plc")
                return

    def send_plc_cmd(self,msg):
        cmd_slisde_dis = msg.slide_dis
        cmd_arm_dis = msg.arm_dis
        cmd_arm_dis = 100

        cmd_slisde_dis = struct.pack('f',cmd_slisde_dis)
        cmd_arm_dis = struct.pack('f',cmd_arm_dis)


        cmd_string = b''
        cmd_string += bytes([0xFF])
        for i in cmd_slisde_dis:
            cmd_string += bytes([i])
        for i in cmd_arm_dis:
            cmd_string += bytes([i])
        cmd_string += bytes([0xAA])

        self.enable_plc_receive = True
        # cmd_string = "%d %d"%(cmd_slisde_dis,cmd_arm_dis)
        # cmd_string = bytes(cmd_string,encoding="utf8")
        print(cmd_string)

        self.send_plc(cmd_string)


    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.receive_plc_func()
            self.receive_stm32_func()
            

if __name__ == '__main__':
    node1 = Com()
    node1.MainLoop()
