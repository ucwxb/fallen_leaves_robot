#!/usr/bin/python3
#coding:utf-8
import socket
import threading
import time
import sys
import cv2
import numpy


class tcp:
    def __init__(self,ip = '192.168.8.216',port = 8888,isserver=True):
        self.isserver=isserver
        self.decimg = None
        self.ip = ip
        self.port = port
        self.address = (self.ip, self.port)
    def start(self):
        if self.isserver:
            self.ReceiveVideo()
        else:
            self.SendVideo()   

    def recvall(self,sock,count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    def ReceiveVideo(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(self.address)
        s.listen(1)
        conn, addr = s.accept()
        print('connect from:'+str(addr))
        self.isReading = False
        while 1:
            # start = time.time()
            length = self.recvall(conn,16)
            stringData = self.recvall(conn, int(length))
            data = numpy.frombuffer(stringData, numpy.uint8)
            self.decimg=cv2.imdecode(data,cv2.IMREAD_COLOR)
            # print("fps:",1/(time.time() - start))
        s.close()
    
       


    def SendVideo(self):
        while 1:
            try:
                sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
                sock.connect(self.address)
                break
            except:
                print("not connect")
                time.sleep(1)
        
        encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),15]
        while 1:
            self.cap = cv2.VideoCapture(0)
            ret, frame = self.cap.read()
            time.sleep(0.01)
            result, imgencode = cv2.imencode('.jpg', frame, encode_param)
            data = numpy.array(imgencode)
            stringData = data.tostring()
            sock.send(str.encode(str(len(stringData)).ljust(16)))
            sock.send(stringData)
        sock.close()
    

if __name__ == '__main__':
    b=tcp(isserver=False)
   
    b.start()
    
