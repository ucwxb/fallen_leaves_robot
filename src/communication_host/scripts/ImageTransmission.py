#!/usr/bin/python3
#coding:utf-8
from my_udp import UDP_Manager
import numpy as np
import cv2


class ImageTransmiter:
    def __init__(self, isServer = False, ip = '', port = 8848):
        self.UDPServer = UDP_Manager(self.Callback, buffSize = 2048, isServer = isServer, port = port)
        if isServer:
            self.targetDict = {}
        else:
            self.targetDict = {}
            self.targetDict[(ip, port)] = 1
        
        self.img_width = 640
        self.img_height = 480
        self.jpegQuality = 20
        self.errImg = np.zeros((self.img_height, self.img_width, 3), np.uint8)
        self.errImg[:,0:200] = [0, 0, 255]
        self.errImg[:,220:420] = [0, 255, 0]
        self.errImg[:,440:640] = [255, 0, 0]
        self.errImgData = cv2.imencode('.jpg', self.errImg, (cv2.IMWRITE_JPEG_QUALITY, self.jpegQuality))[1].tobytes()
        self.UDPServer.Start()
        
    def SetImageQuality(self, width, heigth, quality):
        self.img_width, self.img_height, self.jpegQuality = width, heigth, quality
        
    def Callback(self, data, addr):
        if self.targetDict.get(addr) == None:
            self.targetDict[addr] = 1
            
            print('monitor {} connected.'.format(addr))
        
    def Broadcast(self, img):
        img = cv2.resize(img, (self.img_width, self.img_height))
        data = cv2.imencode('.jpg', img, (cv2.IMWRITE_JPEG_QUALITY, self.jpegQuality))[1].tobytes()
        #print(data)
        if len(data) < 64000:
            for target in self.targetDict.keys():
                self.UDPServer.Send(data, target)
        else:
            for target in self.targetDict.keys():
                self.UDPServer.Send(self.errImgData, target)
        
            
if __name__ == '__main__':
    import time
    imgTransmiter = ImageTransmiter(isServer = True)
    #imgTransmiter.SetImageQuality
    cap = cv2.VideoCapture(0)
    while True:
        _, frame = cap.read()
        imgTransmiter.Broadcast(frame)
        time.sleep(0.03)
