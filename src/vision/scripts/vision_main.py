#!/usr/bin/python3
#coding:utf-8
import rospy
import cv2
import os
import numpy as np
from predict_image import detectImage
import json
from vision.msg import leaf_msg,leaf_detect_msg
from CamTrans import CameraTrans
import time
from TCP import tcp
import threading
from communication_host.srv import *
class VisionNode:
    def __init__(self):
    
        rospy.init_node('vision_node', anonymous = True)          #创建节点
        
        self.rate = rospy.Rate(20)
        self.packagePath = rospy.get_param("/pkg_path/vision")
        self.ptPath = os.path.join(self.packagePath,"scripts")
        self.ptPath = os.path.join(self.ptPath,rospy.get_param("/pt_path"))

        self.yolov5Module = detectImage(os.path.join(self.packagePath, self.ptPath))  #加载模型
        self.leaf_detect_topic = rospy.Publisher("/leaf_detect",leaf_detect_msg,queue_size=1)
        # rospy.Service('/vision_service',leaf_detect_srv, self.leaf_detect_src_callback)  #建立服务

        self.cameraInfo = CameraTrans((640, 480), (0.5, 0.4))  #相机位置信息
        cameraX_robot      = rospy.get_param('/camera_relativeLocation/x')
        cameraY_robot      = rospy.get_param('/camera_relativeLocation/y')
        cameraZ_robot      = rospy.get_param('/camera_relativeLocation/z')
        cameraYaw_robot    = rospy.get_param('/camera_relativeLocation/yaw')
        cameraPitch_robot  = rospy.get_param('/camera_relativeLocation/pitch')
        self.cameraInfo.SetLocation(cameraX_robot, cameraY_robot, cameraZ_robot,
                                    cameraYaw_robot, cameraPitch_robot)           #相机和激光雷达在机器人坐标系下使用

        self.cap = cv2.VideoCapture(0)

        # rospy.wait_for_service('/image_trans')
        # self.srv_getImg = rospy.ServiceProxy('/image_trans',image_trans)
        '''
        self.my_tcp = tcp(ip="192.168.8.216") #192.168.8.225
        self.my_tcp_thread = threading.Thread(target=self.my_tcp.start)
        self.my_tcp_thread.start()
        print("TCP is ready")
        '''

    def get_img(self):
        start_time = time.time()  #计算ros传输帧数
        img = self.srv_getImg().img
        img = np.array(img,dtype="uint8")
        img = img.reshape(len(img),1)
        # img = [[x] for x in img]
        img = cv2.imdecode(img,cv2.IMREAD_COLOR)
        end_time  = time.time()  #计算ros传输帧数
        fps = 1/(end_time-start_time)
        # print(fps)
        return img
        
    def leaf_detect_src_callback(self):
        # self.frame = self.get_img()
        _,self.frame = self.cap.read()
        start = time.time()
        detect_res,self.frame = self.yolov5Module.detect(self.frame)  #画box
        leaf_detect_res = leaf_detect_msg()
        # leaf_detect_srv_res = leaf_detect_srvResponse()
        if detect_res is not None and len(detect_res):
            leaf_detect_res.isFind = 1
            for each_leaf in detect_res:
                new_leaf_msg = leaf_msg()
                *xywh, conf, class_index = each_leaf
                new_leaf_msg.class_index = class_index
                new_leaf_msg.conf = conf
                # global_x,global_y,global_z = self.cameraInfo.img2global(xywh[0],xywh[1])
                # new_leaf_msg.x = global_x
                # new_leaf_msg.y = global_y
                # new_leaf_msg.z = global_z
                new_leaf_msg.x = xywh[0]
                new_leaf_msg.y = 480.0-xywh[1]
                new_leaf_msg.z = 0
                leaf_detect_res.res.append(new_leaf_msg)
        else:
            leaf_detect_res.isFind = 0
        self.leaf_detect_topic.publish(leaf_detect_res)
        # cv2.imshow("win",self.frame)
        # print(1/(time.time() - start))
        # if cv2.waitKey(1) & 0xFF == ord('q'):
            # return

    def leaf_detect_src(self):
        if self.my_tcp.decimg is None:
            return
        # start = time.time()
        self.frame = self.my_tcp.decimg.copy()

        detect_res,self.frame = self.yolov5Module.detect(self.frame)  #画box
        if detect_res is not None and len(detect_res):
            for each_leaf in detect_res:
                *xywh, conf, class_index = each_leaf
        # fps  = 1/(time.time() - start)
        # cv2.imshow("main_win",self.frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
            # return
        # print(int(fps))
        
    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.leaf_detect_src_callback()
            #self.leaf_detect_src()
            
if __name__ == '__main__':
    visionNode = VisionNode()
    visionNode.MainLoop()
