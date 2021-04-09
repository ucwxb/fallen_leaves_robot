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
# from TCP import tcp
from UDP import UDP_Manager
import threading
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
class VisionNode:
    def __init__(self):
    
        rospy.init_node('vision_node', anonymous = True)          #创建节点
        
        self.rate = rospy.Rate(20)
        self.packagePath = rospy.get_param("/pkg_path/vision")
        self.ptPath = os.path.join(self.packagePath,"scripts")
        self.ptPath = os.path.join(self.ptPath,rospy.get_param("/pt_path"))

        self.yolov5Module = detectImage(os.path.join(self.packagePath, self.ptPath))  #加载模型
        self.leaf_detect_topic = rospy.Publisher("/leaf_detect",leaf_detect_msg,queue_size=1)

        self.cap = cv2.VideoCapture(rospy.get_param("/cam_index"))
        
        self.frame = np.zeros((640,480))


        # self.bridge = CvBridge()
        # self.leaf_image_topic = rospy.Publisher("/leaf_image", Image,queue_size=1)
        # rospy.wait_for_service('/image_trans')
        # self.srv_getImg = rospy.ServiceProxy('/image_trans',image_trans)

        # self.my_tcp = tcp() #192.168.8.225
        self.jpegQuality = 20
        self.img_width = 640
        self.img_height = 480
        self.errImg = np.zeros((self.img_height, self.img_width, 3), np.uint8)
        self.errImg[:,0:200] = [0, 0, 255]
        self.errImg[:,220:420] = [0, 255, 0]
        self.errImg[:,440:640] = [255, 0, 0]
        self.errImgData = cv2.imencode('.jpg', self.errImg, (cv2.IMWRITE_JPEG_QUALITY, self.jpegQuality))[1].tobytes()

        self.udp  = UDP_Manager(self.rev_data_cb)
        self.udp.Start()
        # print("TCP is ready")
    
    def rev_data_cb(self,recvData, recvAddr):
        return
        
    def leaf_detect_func(self):
        _,self.frame = self.cap.read()
        self.frame = cv2.flip(self.frame,0)  #翻转
        try:
            detect_res,self.frame = self.yolov5Module.detect(self.frame)  #画box
        except:
            return
        leaf_detect_res = leaf_detect_msg()
        
        if detect_res is not None and len(detect_res):
            leaf_detect_res.isFind = 1
            
            for each_leaf in detect_res:
                new_leaf_msg = leaf_msg()
                *xywh, conf, class_index = each_leaf
                new_leaf_msg.class_index = int(class_index)
                new_leaf_msg.conf = conf
                new_leaf_msg.x = 480.0-xywh[1]
                new_leaf_msg.y = xywh[0] - 320.0
                new_leaf_msg.z = 0
                leaf_detect_res.res.append(new_leaf_msg)
        else:
            leaf_detect_res.isFind = 0
            leaf_detect_res.res = []
        
        data = cv2.imencode('.jpg', self.frame, (cv2.IMWRITE_JPEG_QUALITY, self.jpegQuality))[1].tobytes()
        if len(data) < 64000:
            self.udp.Send(data)
        else:
            self.udp.Send(self.errImgData)
        self.leaf_detect_topic.publish(leaf_detect_res)
        # cv2.imshow("win",self.frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     return

        
    def MainLoop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.leaf_detect_func()
            #self.leaf_detect_src()
            
if __name__ == '__main__':
    visionNode = VisionNode()
    visionNode.MainLoop()
