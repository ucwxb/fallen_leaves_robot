import time

PS_HEAD = 1
PS_LENGTH1 = 2
PS_LENGTH2 = 3
PS_PORT_TYPE_LEVEL = 4
PS_ID = 5
PS_DATA = 6

'''
数据包格式

字节数      数据        说明
1           0xFF       包头
2           0x         字节长度（数据部分） 0~65535,第一个字节为高八位，第二个字节为低八位
1           0x         该字节低四位表示数据类型(Port)，即数据用哪个PortCallback处理
                       高两位表示数据包类型(Type)，00表示该包是需要重传和确认的数据包
                                           01表示该包是无需重传和确认的数据包
                                           10表示该包是确认包
                        中间两位表示包优先级(level) 00 > 01 > 10 > 11
1           0x          数据包序号，0~255
n           ...         data部分
1           0x          校验和，对数据部分累加取低八位

'''

class MyPacket():
    def __init__(self, output):
        '''
        创建对象时传入输出函数output(data), data为字节串
        '''
        self.State = PS_HEAD
        self.Len = 0
        self.CurLen = 0
        self.Id = 0
        self.Type = 0
        self.Port = 0
        self.CheckSum = 0
        self.Level = 0

        self.OverTime = 0.1

        self.SendId = [0, 0, 0, 0]
        self.RecvFlag = [1, 1, 1, 1]
        self.Time = [0, 0, 0, 0]
        self.ReTimes = [0, 0, 0, 0]
        self.LastId = [0, 0, 0, 0]

        self.SendBuff = [[], [], [], []]
        self.Data = []

        self.PortCallback = {}
        self.TypeCallback = {}
        self.TypeCallback[0] = self.Type0Callback
        self.TypeCallback[1] = self.Type1Callback
        self.TypeCallback[2] = self.Type2Callback
        self.Output = output

    def setPortCallback(self, portcallback, port):
        '''
        @brief  对Port端口设置回调函数
        @param  Packet结构体地址
                Port:要设置的端口
                portcallback:要设置的该端口的回调函数(需用户传入) 
                def portcallback(data), data为相应端口收到的数据，list类型
        @retval 无
        '''
        self.PortCallback[port] = portcallback

    
    def Receiver_Handler(self, bytestream):
        '''
        @brief  用于处理接收到的数据包
        @param  bytestream:收到的字节串
        @retval 调用相应的Callback处理数据
        '''
        for rx in bytestream:
            if self.State == PS_HEAD:
                if rx == 0xFF:
                    self.State = PS_LENGTH1
            elif self.State == PS_LENGTH1:
                self.Len |= rx
                self.Len <<= 8
                self.State = PS_LENGTH2
            elif self.State == PS_LENGTH2:
                self.Len |= rx
                self.State = PS_PORT_TYPE_LEVEL
            elif self.State == PS_PORT_TYPE_LEVEL:
                self.Type = rx >> 6
                self.Port = rx & 0x0F
                self.Level = (rx >> 4) & 0x03
                self.State = PS_ID
            elif self.State == PS_ID:
                self.Id = rx
                self.State = PS_DATA
            elif self.State == PS_DATA:
                if self.CurLen < self.Len:
                    self.Data.append(rx)
                    self.CheckSum += rx
                    self.CurLen += 1
                elif rx == self.CheckSum % 256:
                    self.TypeCallback.get(self.Type)(self.Data, self.Id, self.Port, self.Level)
                    self.LastId[self.Level] = self.Id
                    self.State = PS_HEAD
                    self.Data = []
                    self.Level = 0
                    self.Type = 0
                    self.Port = 0
                    self.CheckSum = 0
                    self.CurLen = 0
                    self.Len = 0
                else:
                    self.State = PS_HEAD
                    self.Data = []
                    self.Type = 0
                    self.Level = 0
                    self.Port = 0
                    self.CheckSum = 0
                    self.CurLen = 0
                    self.Len = 0

    def SendData(self, data, len, type, port, level):
        '''
        @brief  type:00:将数据加入缓冲区
                     01:直接发送数据
        @param  data:要发送的数据,list类型或字节串
                len:发送数据的长度(0~65535)
                type:发送数据包的类型
                port:发送数据选择的处理端口
                level:数据包的优先级
        @retval 无
        '''
        CheckSum = 0
        send = b''
        send += bytes([0xFF])
        send += bytes([len >> 8])
        send += bytes([len & 0xFF])
        send += bytes([(type << 6) | port | (level << 4)])
        if type == 0:
            self.SendId[level] += 1
            self.SendId[level] %= 256
            send += bytes([self.SendId[level]])
        else:
            send += bytes([0])
        for i in data:
            send += bytes([i])
            CheckSum += i
        CheckSum %= 256
        send += bytes([CheckSum])
        if type == 0:
            self.SendBuff[level].append(send)
        else:
            print("发送传感字节串为：", send)
            self.Output(send)

    
    def Update(self, level):
        '''
        @brief  对发送队列进行更新(检查超时、重发、更新send缓冲区并发送下一个数据)
        @param  level：指定需要更新的优先级缓冲区
        @retval 同一数据重发超过三次，返回1，正常情况下返回0
        '''
        flag = 4
        i = 3
        while i >= 0:
            if self.RecvFlag[i] == 0:
                flag = i
            i -= 1
        dt = time.time() - self.Time[level]
        #第一个条件判断有无超时或是否有更高优先级或同级还未回复，第二个条件判断缓冲区是否为空
        if ((dt > self.OverTime and self.RecvFlag[level] == 0) or level < flag) and len(self.SendBuff[level]) != 0:
            if self.RecvFlag[level] == 0:
                print('-----------超时-----------')
                self.ReTimes[level] += 1
            self.Output(self.SendBuff[level][0])
            self.Time[level] = time.time()
            self.RecvFlag[level] = 0
            if self.ReTimes[level] > 3:
                return 1
        return 0
    
    def SetTimeOut(self, timeout):
        self.OverTime = timeout

    def Type0Callback(self, data, id, port, level):
        if id != self.LastId[level]:
            print("-----------收到type0数据，发回复-----------")
            self.PortCallback.get(port)(data)
            self.SendData(data, len(data), 2, 0, level)
        else:
            print("-----------收到重复type0数据，只发回复-----------")
            self.SendData(data, len(data), 2, 0, level)

    def Type1Callback(self, data, id, port, level):
        self.PortCallback.get(port)(data)

    def Type2Callback(self, data, id, port, level):
        #self.OverTime = (time.time() - self.Time) * 3  #更新超时时间
        if (len(self.SendBuff[level]) == 0):
            return
        n = 0
        last_data = []
        for i in self.SendBuff[level][0]:
            n += 1
            if n >= 6 and n <= len(self.SendBuff[level][0]) - 1:
                last_data.append(i)
        if (last_data != data):
            return
        self.RecvFlag[level] = 1
        print('-----------收到回复-----------')
        self.ReTimes[level] = 0
        self.SendBuff[level].pop(0)