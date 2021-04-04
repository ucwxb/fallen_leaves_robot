#!/usr/bin/python3
#coding:utf-8
import serial
import Jetson.GPIO
import time
import threading
import rospy
from std_msgs.msg import String
class Com:
    def __init__(self):
        rospy.init_node('communication_node', anonymous = True)
        self.pkg_path = rospy.get_param("/pkg_path/communication_scm")
        self.serial_path = rospy.get_param("/serial_path")
        self.serial_rate = int(rospy.get_param("/serial_rate"))

        self.rate = rospy.Rate(20)

        rospy.Subscriber("/send_stm32",Int32,self.send_stm32)
        self.receive_stm32 = rospy.Publisher("/receive_stm32", String,queue_size=1)

        self.init_serial()
        self.init_threading()
    
    def init_serial(self):
        while(1):
            try:
                self.ser = serial.Serial(self.serial_path, self.serial_rate,timeout=1)
                break
            except:
                print("serial time out")
            time.time(1)
    
    def init_threading(self):
        self.t = threading.Thread(target=self.receive_stm32_func)
    
    def receive_stm32_func(self):
        while(1):
            if self.ser.isOpen():
                res = str(self.ser.readall())
                self.receive_stm32.publish(res)
                print(res)
            else:
                print("ser is cloesd")

    def send_stm32(self,msg):
        data = msg.data
        self.ser.write(info)


    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            

if __name__ == '__main__':
    node1 = Com()
    node1.MainLoop()
