#!/usr/bin/python3
#coding:utf-8
import serial
import Jetson.GPIO
import time
import threading
import rospy
from std_msgs.msg import String,Int32
class Com:
    def __init__(self):
        rospy.init_node('communication_node', anonymous = True)
        self.pkg_path = rospy.get_param("/pkg_path/communication_scm")
        self.serial_path = rospy.get_param("/serial_path")
        self.plc_serial_path = rospy.get_param("/plc_serial_path")
        self.serial_rate = int(rospy.get_param("/serial_rate"))

        self.rate = rospy.Rate(20)

        rospy.Subscriber("/send_stm32",Int32,self.send_stm32)
        self.receive_stm32 = rospy.Publisher("/receive_stm32", String,queue_size=1)

        rospy.Subscriber("/send_plc",Int32,self.send_stm32)
        self.receive_plc = rospy.Publisher("/receive_plc", String,queue_size=1)

        self.init_serial()
        self.init_threading()
    
    def init_serial(self):
        while(1):
            try:
                self.ser = serial.Serial(self.serial_path, self.serial_rate,timeout=1)
                break
            except:
                print("stm32 serial time out")
            time.sleep(1)
        while(1):
            try:
                self.plc_ser = serial.Serial(self.plc_serial_path, self.serial_rate,timeout=1)
                break
            except:
                print("plc serial time out")
            time.sleep(1)
    
    def init_threading(self):
        self.t = threading.Thread(target=self.receive_stm32_func)
        self.t.start()
        self.plc_t = threading.Thread(target=self.receive_plc_func)
        self.plc_t.start()

    def send_stm32(self,msg):
        data = msg.data
        self.ser.write(info)

    def receive_stm32_func(self):
        while(1):
            if self.ser.isOpen():
                res = str(self.ser.readall())
                self.receive_stm32.publish(res)
                print(res)
            else:
                print("ser is cloesd")

    def send_plc(self,msg):
        data = msg.data
        self.plc_ser.write(info)
    
    def receive_plc_func(self):
        while(1):
            if self.plc_ser.isOpen():
                res = str(self.plc_ser.readall())
                self.receive_plc.publish(res)
                print(res)
            else:
                print("ser is cloesd")




    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            

if __name__ == '__main__':
    node1 = Com()
    node1.MainLoop()
