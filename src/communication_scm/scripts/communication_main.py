#!/usr/bin/python3
#coding:utf-8
import serial
import Jetson.GPIO
import time
import threading
import rospy
from std_msgs.msg import String,Int32
from communication_scm.msg import plc_cmd,stm_vel_cmd
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
        rospy.Subscriber("/send_plc_cmd",plc_cmd,self.send_plc_cmd)

        self.init_serial()
        self.init_threading()
    
    def init_serial(self):
        times = 0
        while(1):
            try:
                if times >= 2:
                    print("failed to connect to stm32")
                    break
                self.ser = serial.Serial(self.serial_path, self.serial_rate,timeout=1)
                break
            except:
                times += 1
                print("stm32 serial time out")
            time.sleep(1)
        times = 0
        while(1):
            try:
                if times >= 0:
                    print("failed to connect to plc")
                    break
                self.plc_ser = serial.Serial(self.plc_serial_path, self.serial_rate,timeout=1)
                break
            except:
                times += 1
                print("plc serial time out")
            time.sleep(1)
    
    def init_threading(self):
        self.t = threading.Thread(target=self.receive_stm32_func)
        self.t.start()
        self.plc_t = threading.Thread(target=self.receive_plc_func)
        self.plc_t.start()

    def send_stm32_cb(self,msg):
        data = msg.data
        self.send_stm32(data)

    def send_stm32(self,string):
        self.ser.write(string)

    def receive_stm32_func(self):
        while(1):
            if self.ser.isOpen():
                res = str(self.ser.readall())
                self.receive_stm32.publish(res)
                print(res)
            else:
                print("ser is cloesd")
    
    def send_stm32_vel(self,msg):
        vel_x = msg.x
        vel_y = msg.y
        vel_yaw = msg.yaw
        vel_x = struct.pack('f',vel_x)
        vel_y = struct.pack('f',vel_y)
        vel_yaw = struct.pack('f',vel_yaw)
        msg_type = msg.type
        cmd_string = b''
        cmd_string += bytes([0xFF])
        cmd_string += bytes([msg_type])
        for i in vel_x:
            cmd_string += i
        for i in vel_y:
            cmd_string += i
        for i in vel_yaw:
            cmd_string += i
        self.send_stm32(cmd_string)

    def send_plc_cb(self,msg):
        data = msg.data
        self.send_plc(data)
    
    def send_plc(self,string):
        self.plc_ser.write(string)
    
    def receive_plc_func(self):
        while(1):
            if self.plc_ser.isOpen():
                res = str(self.plc_ser.readall())
                self.receive_plc.publish(res)
                print(res)
            else:
                print("ser is cloesd")

    def send_plc_cmd(self,msg):
        cmd_type = msg.type
        cmd_fan_status = msg.fan_status
        cmd_slide_status = msg.slide_status
        cmd_slisde_dis = msg.slide_dis
        cmd_string = "type:"
        self.send_plc(cmd_string)


    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            

if __name__ == '__main__':
    node1 = Com()
    node1.MainLoop()
