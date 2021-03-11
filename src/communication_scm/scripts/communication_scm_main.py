#!/usr/bin/python3
#coding:utf-8
import rospy
from communication_scm.msg import *
from packet import MyPacket
import struct
import serial

class TransScm:
    def __init__(self):
        rospy.init_node('communication_scm_node', anonymous=True)
        self.rate = rospy.Rate(20)  #50HZ
        self.Port = "/dev/ttyUSB1"    #串口端口
        self.Bps = 115200    #串口通信波特率
        self.TimeOut = 0.1    #串口通信超时时间
        self.ser = serial.Serial(self.Port, self.Bps, timeout=self.TimeOut)     #打开串口
        self.packet = MyPacket(self.ser.write)

        self.gps_location_pub = rospy.Publisher('/gps_location', gps_location_msg, queue_size=1)   
          

    def gps_location_cb(self, data):
        '''定位信息port回调函数，发布话题'''
        data = bytes(data)
        lmsg = location_msg()
        vmsg = velocity_msg()
        (lmsg.x, lmsg.y, lmsg.yaw, vmsg.x, vmsg.y, vmsg.yaw) = struct.unpack('=ffffff', data)
        self.location_pub.publish(lmsg)
        self.velocity_pub.publish(vmsg)



    def main(self):
        #为对应端口设置接收回调函数
        self.packet.setPortCallback(self.gps_location_cb,4)
        #主循环
        while not rospy.is_shutdown():
            #self.Location_Err_Send()   #发误差
            UnNormal1 = self.packet.Update(0)     #更新发送缓存区
            UnNormal2 = self.packet.Update(1)     #更新发送缓存区
            UnNormal3 = self.packet.Update(2)     #更新发送缓存区
            UnNormal4 = self.packet.Update(3)     #更新发送缓存区
            #接收串口数据并解包执行相应操作
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                #print(data)
                self.packet.Receiver_Handler(data)
            self.rate.sleep()
    
if __name__ == "__main__":
    trans = TransScm()
    trans.main()

