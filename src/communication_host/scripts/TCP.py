#!/usr/bin/python3
#coding:utf-8
import socket
import threading
import time
import sys
import cv2
import numpy


class tcp:
    def __init__(self,ip = '192.168.8.225',port = 8888,isserver=True):
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
            start = time.time()#用于计算帧率信息
            length = self.recvall(conn,16)#获得图片文件的长度,16代表获取长度
            stringData = self.recvall(conn, int(length))#根据获得的文件长度，获取图片文件
            data = numpy.frombuffer(stringData, numpy.uint8)#将获取到的字符流数据转换成1维数组
            self.decimg=cv2.imdecode(data,cv2.IMREAD_COLOR)#将数组解码成图像
            #print(decimg)
            #cv2.imshow('SERVER',decimg)#显示图像
            end = time.time()
            seconds = end - start
            fps  = 1/seconds
            print(int(fps))
            # conn.send(bytes(str(int(fps)),encoding='utf-8'))
            #k = cv2.waitKey(10)&0xff
            #if k == 27:
            #    break
        s.close()
    #cv2.destroyAllWindows()
    
       


    def SendVideo(self):
        self.cap = cv2.VideoCapture(0)
        try:
            sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            sock.connect(self.address)
        except socket.error as msg:
            print(msg)
            return
        ret, frame = self.cap.read()
        encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),15]  #压缩图片
        while ret:
            time.sleep(0.01)
            result, imgencode = cv2.imencode('.jpg', frame, encode_param)
            #建立矩阵
            data = numpy.array(imgencode)
            #将numpy矩阵转换成字符形式，以便在网络中传输
            stringData = data.tostring()
            
            #先发送要发送的数据的长度
            #ljust() 方法返回一个原字符串左对齐,并使用空格填充至指定长度的新字符串
            sock.send(str.encode(str(len(stringData)).ljust(16)))
            #发送数据
            sock.send(stringData)
            #读取服务器返回值
            receive = sock.recv(1024)
            if len(receive):
                print(str(receive,encoding='utf-8'))
            #读取下一帧图片
            ret, frame = self.cap.read()
            if cv2.waitKey(10) == 27:
                break
        sock.close()       
    

if __name__ == '__main__':
    b=tcp(isserver=False)
   
    b.start()
    
