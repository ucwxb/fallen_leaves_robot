#!/usr/bin/python3
import time
import math
from Arm_Lib import Arm_Device
import rospy
from std_msgs.msg import UInt32,UInt32MultiArray,Int32MultiArray
from arm_control.msg import * 
class ArmCon:
    def __init__(self):
        rospy.init_node('gesture_recogniton', anonymous=True)
        self.rate = rospy.Rate(20)
        self.Arm = Arm_Device()
        time.sleep(.1)
        # self.angle_list = [angle1,angle2,angle3,angle4]
        self.angle_list = rospy.get_param("/angle_list")
        self.current_angle = [0,0,0,0,0,0]
        rospy.Subscriber("/servo_angle_control_topic",UInt32,self.servo_angle_control_topic_cb)
        rospy.Subscriber("/manual",manual,self.servo_manual_control_topic_cb)
        self.read_servo_angle_topic = rospy.Publisher("/read_servo_angle_topic",Int32MultiArray,queue_size=1)
    
    def ctrl_all_servo(self,angle, s_time = 500):
        print(123)
        self.Arm.Arm_serial_servo_write6(angle[0], angle[1], angle[2], angle[3], angle[4], angle[5], s_time)
        time.sleep(s_time/1000)

    def servo_angle_control_topic_cb(self,msg):
        Number = msg.data
        if Number==1:
            self.ctrl_all_servo(self.angle_list[0],s_time=500)
            self.ctrl_all_servo(self.angle_list[1],s_time=500)
        if Number==2:
            self.ctrl_all_servo(self.angle_list[2],s_time=500)
            self.ctrl_all_servo(self.angle_list[3],s_time=500)

    def servo_manual_control_topic_cb(self,msg):
        print(msg)
        angle1 = msg.angle1
        angle2 = msg.angle2
        angle3 = msg.angle3
        angle4 = msg.angle4
        angle5 = msg.angle5
        angle6 = msg.angle6
        msg_angle_list = []
        msg_angle_list.append(angle1)
        msg_angle_list.append(angle2)
        msg_angle_list.append(angle3)
        msg_angle_list.append(angle4)
        msg_angle_list.append(angle5)
        msg_angle_list.append(angle6)
        self.ctrl_all_servo(msg_angle_list,s_time=500)

    def read_servo_angle(self):
        for i in range(6):
            self.current_angle[i] = self.Arm.Arm_serial_servo_read(i+1)
            if self.current_angle[i] == None:
                self.current_angle[i] = -1
            time.sleep(.1)
        res = Int32MultiArray()
        res.data = self.current_angle
        self.read_servo_angle_topic.publish(res)
        time.sleep(0.3)
    

    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.read_servo_angle()

if __name__ == '__main__':
    node1 = ArmCon()
    node1.MainLoop()
