import socket
import time
import struct
import threading

class UDP_Manager:
    def __init__(self, callback, buffSize = 2048, isServer = False, port = 8083, frequency = 50):
        self.callback = callback
        self.buffSize = buffSize
        self.isServer = isServer
        self.interval = 1.0 / frequency
        
        self.ip = ''
        self.GetServerIP()
        self.port = port
        self.addr = (self.ip, self.port)
        self.running = False

    def GetServerIP(self):
        #查询本机ip地址
        try:
            s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            s.connect(('8.8.8.8',80))
            self.ip = s.getsockname()[0]
        finally:
            s.close()

    def Start(self):
        self.sockUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  #UDP协议
        if self.isServer:
            self.sockUDP.bind(self.addr)
            self.roleName = 'Server'
        else:
            self.roleName = 'Client'
        #获取本机名称
        self.myname = socket.getfqdn(socket.gethostname())
        print(self.roleName, '(UDP) name：', self.myname)
        #获取本机ip
        if self.isServer:
            print(self.roleName, '(UDP) at:', self.ip, ':', self.port)
        self.running = True
        self.thread = threading.Thread(target = self.Receive, args=())
        self.thread.start()  #打开收数据的线程
    
    def Receive(self):
        print(self.roleName, '(UDP) is running...\n')
        while self.running:
            time.sleep(self.interval)
            while self.running:
                try:
                    recvData, recvAddr = self.sockUDP.recvfrom(self.buffSize) #等待接受数据
                    print(recvData)
                except:
                    break
                if not recvData:
                    break
                self.callback(recvData, recvAddr)
            
    def Send(self, data, addr):
        self.sockUDP.sendto(data, addr)

    def Close(self):
        self.running = False
        

if __name__ == '__main__':
    def callback(x, y):
        print(x, y)
    
    ip = '192.168.1.41'
    a = UDP_Manager(callback, buffSize = 2048, isServer = True, port = 8848)
    b = UDP_Manager(callback, buffSize = 2048, isServer = False, port = 8848)
    a.Start()
    b.Start()
