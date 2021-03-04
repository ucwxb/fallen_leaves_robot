#!/usr/bin/python3
#coding:utf-8
import rospy
import cv2
import os
from predict_image import detectImage
import json
from vision.srv import *
from vision.msg import leaf_msg
from CamTrans import CameraTrans
from ImageTransmission import ImageTransmiter
class VisionNode:
    def __init__(self):
    
        rospy.init_node('vision_node', anonymous = True)          #创建节点
        
        self.rate = rospy.Rate(20)

        self.cap = cv2.VideoCapture(rospy.get_param("/cam_index"))  #临时调试用
        
        #self.cap.set(3,480) #调整相机画幅大小，上位机不可用
        #self.cap.set(4,640)
        self.packagePath = rospy.get_param("/pkg_path/vision")
        self.ptPath = self.packagePath+rospy.get_param("/pt_path")
        self.yolov5Module = detectImage(os.path.join(self.packagePath, self.ptPath))  #加载模型
        rospy.Service('/vision_service',leaf_detect_srv, self.leaf_detect_src_callback)  #建立服务

        self.cameraInfo = CameraTrans((640, 480), (0.5, 0.4))  #相机位置信息
        cameraX_robot      = rospy.get_param('/camera_relativeLocation/x')
        cameraY_robot      = rospy.get_param('/camera_relativeLocation/y')
        cameraZ_robot      = rospy.get_param('/camera_relativeLocation/z')
        cameraYaw_robot    = rospy.get_param('/camera_relativeLocation/yaw')
        cameraPitch_robot  = rospy.get_param('/camera_relativeLocation/pitch')
        self.cameraInfo.SetLocation(cameraX_robot, cameraY_robot, cameraZ_robot,
                                    cameraYaw_robot, cameraPitch_robot)           #相机和激光雷达在机器人坐标系下使用


        # self.computer_img_transmit = ImageTransmiter(isServer=True,ip='192.168.43.155',port=8848)
    def leaf_detect_src_callback(self, data):
        
        _, self.frame = self.cap.read()
        detect_res = self.yolov5Module.detect(self.frame)
        leaf_detect_srv_res = leaf_detect_srvResponse()
        if detect_res is not None and len(detect_res):
            leaf_detect_srv_res.isFind = 1
            for each_leaf in detect_res:
                new_leaf_msg = leaf_msg()
                *xywh, conf, class_index = each_leaf
                new_leaf_msg.class_index = class_index
                new_leaf_msg.conf = conf
                global_x,global_y,global_z = self.cameraInfo.img2global(xywh[0],xywh[1])
                new_leaf_msg.x = global_x
                new_leaf_msg.y = global_y
                new_leaf_msg.z = global_z
                leaf_detect_srv_res.res.append(new_leaf_msg)
        else:
            leaf_detect_srv_res.isFind = 0
        return leaf_detect_srv_res
        
    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            
            
if __name__ == '__main__':
    visionNode = VisionNode()
    visionNode.MainLoop()
