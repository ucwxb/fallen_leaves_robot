#!/usr/bin/python3
import time
import math
from Arm_Lib import Arm_Device
import rospy
from std_msgs.msg import UInt32,UInt32MultiArray,Int32MultiArray
from arm_control.msg import * 
from communication_scm.msg import *
class ArmCon:
    def __init__(self):
        rospy.init_node('gesture_recogniton', anonymous=True)
        self.rate = rospy.Rate(20)
        self.Arm = Arm_Device()
        time.sleep(.1)
        # self.angle_list = [angle1,angle2,angle3,angle4]
        self.angle_list = rospy.get_param("/angle_list")
        self.angle_template = rospy.get_param("/angle_template")
        self.current_angle = [0,0,0,0,0,0]
        self.initial_angle=[]
        self.chazhi_angle=[]
        self.arm_assert_num = rospy.get_param("/arm_assert_num")
        rospy.Subscriber("/servo_angle_control_topic",UInt32,self.new_servo_angle_control_topic_cb)
        rospy.Subscriber("/manual",manual,self.servo_manual_control_topic_cb)
        self.read_servo_angle_topic = rospy.Publisher("/read_servo_angle_topic",Int32MultiArray,queue_size=1)
        self.ctrl_all_servo([90, 180, 90, 0, 80, 100])

        self.send_plc_cmd = rospy.Publisher("/send_plc_cmd",plc_plate_cmd,queue_size=1)
    
    def ctrl_all_servo(self,angle, s_time = 500):
        self.Arm.Arm_serial_servo_write6(angle[0], angle[1], angle[2], angle[3], angle[4], angle[5], s_time)
        time.sleep(s_time/1000)

    def arm_assert(self,first_pos,second_pos):
        assert_angle = first_pos.copy()
        for i in range(self.arm_assert_num):
            for j in range(6):
                assert_angle[i] += (second_pos[i]-first_pos[i])/6
            self.ctrl_all_servo(assert_angle)
    
    def new_servo_angle_control_topic_cb(self,msg):
        Number = msg.data
        self.initial_angle=self.current_angle.copy()
        self.target_angle_list = self.angle_template[Number]
        for i in range(len(self.target_angle_list)):
            self.send_plc_cmd.publish(plc_plate_cmd(100,-20))
            time.sleep(2)
            if i == 0:
                self.arm_assert(self.initial_angle,self.target_angle_list[i])
            else:
                self.arm_assert(self.target_angle_list[i-1],self.target_angle_list[i])
            time.sleep(2)
            self.send_plc_cmd.publish(plc_plate_cmd(100,-10))

    def  arm_chazhi(self,n,m):
        if n==0:
            self.chazhi_angle=self.initial_angle.copy()
            for num in range(0,6):
                for i in range(0,6):
                    self.chazhi_angle[i]=self.chazhi_angle[i]+(self.angle_list[m][i]-self.initial_angle[i])/6
                print(self.chazhi_angle)
                self.ctrl_all_servo(self.chazhi_angle,s_time=500)

        else:
            self.chazhi_angle=self.angle_list[m-1].copy()
            self.chazhi_angle[5]=180
            for num in range(0,20):
                for i in range(0,5):
                    self.chazhi_angle[i]=self.chazhi_angle[i]+(self.angle_list[m][i]-self.angle_list[m-1][i])/20
                print(self.chazhi_angle)
                self.ctrl_all_servo(self.chazhi_angle,s_time=50)


    def servo_angle_control_topic_cb(self,msg):
        Number = msg.data
        self.initial_angle=self.current_angle.copy()
        if Number==1:
            self.send_plc_cmd.publish(plc_plate_cmd(100,-20))
            time.sleep(2)
            self.arm_chazhi(0,0)
            self.Arm.Arm_serial_servo_write(6,180,1500)
            time.sleep(2)
            self.arm_chazhi(1,1)
            self.Arm.Arm_serial_servo_write(6,35,1500)
            self.send_plc_cmd.publish(plc_plate_cmd(100,-10))

        if Number==2:
            self.send_plc_cmd.publish(plc_plate_cmd(100,-20))
            time.sleep(2)
            self.arm_chazhi(0,2)
            self.Arm.Arm_serial_servo_write(6,180,1500)
            time.sleep(2)
            self.arm_chazhi(1,3)
            self.Arm.Arm_serial_servo_write(6,35,1500)
            self.send_plc_cmd.publish(plc_plate_cmd(100,-10))

    def servo_manual_control_topic_cb(self,msg):
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
