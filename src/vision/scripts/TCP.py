#!/usr/bin/python3
#coding:utf-8
import socket
import threading
import time
import sys
import cv2
import numpy


class tcp:
    def __init__(self,isserver=True):
        self.isserver=isserver
        self.decimg = None
        

    def start(self):
        if self.isserver:
            self.ReceiveVideo()
        else:
            self.SendVideo()   

    def ReceiveVideo(self):
        #IP地址'0.0.0.0'为等待客户端连接
        address = ('192.168.8.225', 8888)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(address)开始监听TCP传入连接开始监听TCP传入连接。参数指定在拒绝连接之前，操作系统可以挂起的最大连接数量。该值至少为1，大部分应用程序设为5就可以了。。参数指定在拒绝连接之前，操作系统可以挂起的最大连接数量。该值至少为1，大部分应用程序设为5就可以了。
    #开始监听TCP传入连接。参数指定在拒绝连接之前，操作系统可以挂起的最大连接数量。该值至少为1，大部分应用程序设为5就可以了。
        s.listen(1)       #接受TCP连接并返回（conn,address）,其中conn是新的套接字对象，可以用来接收和发送数据。addr是连接客户端的地址。
    #没有连接则等待有连接
        conn, addr = s.accept()
        print('connect from:'+str(addr))
        def recvall(sock,count):
            buf = b''#buf是一个byte类型
            while count:
                    #接受TCP套接字的数据。数据以字符串形式返回，count指定要接收的最大数据量.
                newbuf = sock.recv(count)
                if not newbuf: return None
                buf += newbuf
                count -= len(newbuf)
            return buf     
    
        while 1:
            start = time.time()#用于计算帧率信息
            length = recvall(conn,16)#获得图片文件的长度,16代表获取长度
            stringData = recvall(conn, int(length))#根据获得的文件长度，获取图片文件
            data = numpy.frombuffer(stringData, numpy.uint8)#将获取到的字符流数据转换成1维数组
            self.decimg=cv2.imdecode(data,cv2.IMREAD_COLOR)#将数组解码成图像
            #print(decimg)
            #cv2.imshow('SERVER',decimg)#显示图像

            end = time.time()
            seconds = end - start
            fps  = 1/seconds
            conn.send(bytes(str(int(fps)),encoding='utf-8'))
            #k = cv2.waitKey(10)&0xff
            #if k == 27:
            #    break
        s.close()
    #cv2.destroyAllWindows()
    
       


    def SendVideo(self):
        #建立sock连接
    #address要连接的服务器IP地址和端口号
        self.cap = cv2.VideoCapture(0)
        address = ('192.168.8.225', 8888)
        try:
            #建立socket对象，参数意义见https://blog.csdn.net/rebelqsp/article/details/22109925
            #socket.AF_INET：服务器之间网络通信 
            #socket.SOCK_STREAM：流式socket , for TCP
            sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            #开启连接
            sock.connect(address)
        except socket.error as msg:
            print(msg)
            sys.exit(1)
    
        #建立图像读取对象

        #读取一帧图像，读取成功:ret=1 frame=读取到的一帧图像；读取失败:ret=0
        ret, frame = capture.read()
        #压缩参数，后面cv2.imencode将会用到，对于jpeg来说，15代表图像质量，越高代表图像质量越好为 0-100，默认95
        encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),15]
    
        while ret:
            #停止0.1S 防止发送过快服务的处理不过来，如果服务端的处理很多，那么应该加大这个值
            time.sleep(0.01)
            #cv2.imencode将图片格式转换(编码)成流数据，赋值到内存缓存中;主要用于图像数据格式的压缩，方便网络传输
            #'.jpg'表示将图片按照jpg格式编码。
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
            if len(receive):print(str(receive,encoding='utf-8'))
            #读取下一帧图片
            ret, frame = capture.read()
            if cv2.waitKey(10) == 27:
                break
        sock.close()       
    

if __name__ == '__main__':
    b=tcp(isserver=False)
   
    b.start()
    
