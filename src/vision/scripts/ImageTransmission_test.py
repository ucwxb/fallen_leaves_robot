#!/usr/bin/python3
#coding:utf-8
from UDP import UDP_Manager
import numpy as np

def Callback(data, addr):
    global img
    data = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    
if __name__ == '__main__':
    import cv2
    import time
    cv2.namedWindow('1', 0)
    UDPServer = UDP_Manager(Callback, port = 8889,buffSize = 64 * 1024)
    UDPServer.Start()
    img = np.zeros((320, 640), np.uint8)
    UDPServer.targetDict[('127.0.0.1',8888)] = 1
    UDPServer.Send(b'req')
    while True:
        cv2.imshow('1', img)
        cv2.waitKey(20)
        
        
    '  图像过大，降低画质后重试  '
