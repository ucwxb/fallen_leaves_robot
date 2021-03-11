#!/usr/bin/python3
#coding:utf-8
import rospy
import cv2
import os
import json
from communication_host.srv import *
class ComHostNode:
    def __init__(self): 
    
        rospy.init_node('communication_host_node', anonymous =True)          #创建节点
        
        self.rate = rospy.Rate(20)

        self.cap = cv2.VideoCapture(rospy.get_param("/cam_index"))  #临时调试用
        
        rospy.Service('/image_trans',image_trans, self.Callback)  #建立服务

    def Callback(self, data):
        _,frame = self.cap.read()
        encoded_image = cv2.imencode(".jpg", frame)[1]
        res = image_transResponse()
        res.img = encoded_image.flatten()
	print(res.img)
        return res
        
    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            
            
if __name__ == '__main__':
    ComHost_Node = ComHostNode()
    ComHost_Node.MainLoop()
