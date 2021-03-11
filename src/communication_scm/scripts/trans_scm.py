#!/usr/bin/python3
#coding:utf-8
import rospy
from main.msg import location_msg, velocity_msg
from communication_scm.msg import relocation_msg, rotation_msg
from packet import MyPacket
import struct
import serial

class trans_scm:
    def __init__(self):
        rospy.init_node('trans', anonymous=True)
        self.rate = rospy.Rate(50)  #50HZ
        self.Port = "/dev/ttyUSB1"    #串口端口
        self.Bps = 115200    #串口通信波特率
        self.TimeOut = 0.1    #串口通信超时时间
        self.ser = serial.Serial(self.Port, self.Bps, timeout=self.TimeOut)     #打开串口
        self.packet = MyPacket(self.ser.write)
        self.relative_loncation_sub = rospy.Subscriber('/relative_location', relocation_msg, self.Relative_Callback)      #订阅壶和机器人相对位置topic
        self.location_err_sub = rospy.Subscriber('/location_err', location_msg, self.LocationErr_Callback)  #定位误差topic
        #发定位信息topic
        self.location_pub = rospy.Publisher('/receive_location', location_msg, queue_size=10)   
        self.velocity_pub = rospy.Publisher('/receive_volocity', velocity_msg, queue_size=10)
          

    def LocatInfo_Callback(self, data):
        '''定位信息port回调函数，发布话题'''
        data = bytes(data)
        lmsg = location_msg()
        vmsg = velocity_msg()
        (lmsg.x, lmsg.y, lmsg.yaw, vmsg.x, vmsg.y, vmsg.yaw) = struct.unpack('=ffffff', data)
        self.location_pub.publish(lmsg)
        self.velocity_pub.publish(vmsg)


    def Relative_Callback(self, msg):
        '''相对位置topic回调函数, 将消息发给下位机'''
        data = struct.pack('f', msg.r)
        data += struct.pack('f', msg.theta)
        self.packet.SendData(data, 8, 0, 3, 3)    


    def LocationErr_Callback(self, msg):
        '''将location_err的三个参数发给下位机'''
        x_err = msg.x
        y_err = msg.y
        yaw_err = msg.yaw
        data = struct.pack('f', x_err)
        data += struct.pack('f', y_err)
        data += struct.pack('f', yaw_err)
        self.packet.SendData(data, 3 * 4, 1, 2, 3)    

    def main(self):
        #为对应端口设置接收回调函数
        self.packet.setPortCallback(self.LocatInfo_Callback, 4)
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
    trans = trans_scm()
    trans.main()       

