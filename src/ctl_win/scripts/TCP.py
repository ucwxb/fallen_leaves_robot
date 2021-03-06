#!/usr/bin/python3
#coding:utf-8
import socket
import threading
import time
import sys
import cv2
import numpy
import rospy

class tcp:
    def __init__(self,ip = '192.168.8.100',port = 8888,is_sender=True):
        self.is_sender=is_sender
        self.decimg = None
        self.ip = ip
        self.port = port
        self.address = (self.ip, self.port)

        if self.is_sender:
            times = 0
            try:
                if times >= 100:
                    return
                self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
                self.s.connect(self.address)
            except:
                print("not connect")
                time.sleep(1)
                times+=1
        else:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.bind(self.address)
            self.s.listen(1)
            self.conn, self.target_addr = self.s.accept()
            self.encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),15]


    def recvall(self,sock,count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    def ReceiveImg(self):
        length = self.recvall(self.conn,16)
        stringData = self.recvall(self.conn, int(length))
        data = numpy.frombuffer(stringData, numpy.uint8)
        return cv2.imdecode(data,cv2.IMREAD_COLOR)

    def SendImg(self,frame):
        result, imgencode = cv2.imencode('.jpg', frame, self.encode_param)
        data = numpy.array(imgencode)
        stringData = data.tostring()
        self.s.send(str.encode(str(len(stringData)).ljust(16)))
        self.s.send(stringData)
    

if __name__ == '__main__':
    b=tcp(isserver=False)
   
    b.start()
    
