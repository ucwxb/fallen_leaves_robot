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
        # rospy.Service('/vision_service',leaf_detect_srv, self.leaf_detect_src_callback)  #建立服务

        self.cameraInfo = CameraTrans((640, 480), (0.5, 0.4))  #相机位置信息
        cameraX_robot      = rospy.get_param('/camera_relativeLocation/x')
        cameraY_robot      = rospy.get_param('/camera_relativeLocation/y')
        cameraZ_robot      = rospy.get_param('/camera_relativeLocation/z')
        cameraYaw_robot    = rospy.get_param('/camera_relativeLocation/yaw')
        cameraPitch_robot  = rospy.get_param('/camera_relativeLocation/pitch')
        self.cameraInfo.SetLocation(cameraX_robot, cameraY_robot, cameraZ_robot,
                                    cameraYaw_robot, cameraPitch_robot)           #相机和激光雷达在机器人坐标系下使用

        self.cap = cv2.VideoCapture(rospy.get_param("/cam_index"))
        
        self.frame = np.zeros((640,480))

        self.index_img = 1

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

        self.udp  = UDP_Manager(self.rev_data_cb,isServer=True)
        self.udp.Start()
        # print("TCP is ready")
    
    def rev_data_cb(self,recvData, recvAddr):
        print(recvData,recvAddr)
        
    def leaf_detect_func(self):
        _,self.frame = self.cap.read()
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
                # global_x,global_y,global_z = self.cameraInfo.img2global(xywh[0],xywh[1])
                # new_leaf_msg.x = global_x
                # new_leaf_msg.y = global_y
                # new_leaf_msg.z = global_z
                new_leaf_msg.x = 480.0-xywh[1]
                new_leaf_msg.y = xywh[0] - 320.0
                new_leaf_msg.z = 0
                leaf_detect_res.res.append(new_leaf_msg)



            # self.my_tcp.SendImg(self.frame)
            cv2.imwrite("%d.jpg"%self.index_img,self.frame)
        else:
            leaf_detect_res.isFind = 0
            leaf_detect_res.res = []
        
        data = cv2.imencode('.jpg', self.frame, (cv2.IMWRITE_JPEG_QUALITY, self.jpegQuality))[1].tobytes()

        if len(data) < 64000:
            target = ('192.168.8.100',8888)
            self.udp.Send(b'123',target)
            self.udp.Send(data,('192.168.8.100',8888))
        else:
            self.udp.Send(self.errImgData,('192.168.8.100',8888))
        
        # img_msg = self.bridge.cv2_to_imgmsg(self.frame, 'rgb8')
        # self.leaf_image_topic.publish(img_msg)
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
