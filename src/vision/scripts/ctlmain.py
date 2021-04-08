#!/usr/bin/python3
#coding:utf-8
import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.Qt import QWidget, QColor, QPixmap, QIcon, QSize, QCheckBox
from PyQt5.QtWidgets import QApplication,QMainWindow
from PyQt5.QtGui import QImage
import rospy
from communication_scm.msg import *
from arm_control.msg import * 
from std_msgs.msg import UInt32,UInt32MultiArray,Int32MultiArray,Empty
from PyQt5.QtCore import Qt,QTimer
# from cv_bridge import CvBridge
# from TCP import tcp
from UDP import UDP_Manager
# from sensor_msgs.msg import Image
import numpy as np
import cv2
class Ui_CtlWin(object):

    def __init__(self, CtlWin):
        self.setupUi(CtlWin)
        self.init_ctl()

    def init_ctl(self):
        rospy.init_node('ctl_win_node')
        self.rospy_rate = 20
        self.rate = rospy.Rate(self.rospy_rate)
        
        self.vel = [0,0,0]
        self._vel = [0,0,0]

        self.front_plate = 0
        self.behind_plate = 0
        self.brush = 0
        self.fan = 0

        self.servo_angle = [0,0,0,0,0,0]
        self._servo_angle = [0,0,0,0,0,0]

        self.send_stm32_vel_topic = rospy.Publisher('/send_stm32_vel', stm_vel_cmd, queue_size=1)
        self.send_brush_topic = rospy.Publisher('/send_stm32_brush', stm_brush_cmd,queue_size=1)
        self.send_fan_topic = rospy.Publisher('/send_stm32_fan', stm_fan_cmd,queue_size=1)
        self.send_plate_topic  = rospy.Publisher('/send_plc_cmd',plc_cmd , queue_size=1)
        self.send_servo_topic = rospy.Publisher('/manual', manual, queue_size=1)
        self.switch_mode_topic = rospy.Publisher('/switch_mode', Empty, queue_size=1)

        rospy.Subscriber('/send_stm32_vel', stm_vel_cmd, self.display_vel)
        rospy.Subscriber('/read_servo_angle_topic', Int32MultiArray, self.display_servo_angle)
        

        self.x_but_add.clicked.connect(self.x_but_add_func)
        self.x_but_minus.clicked.connect(self.x_but_minus_func)
        self.y_but_add.clicked.connect(self.y_but_add_func)
        self.y_but_minus.clicked.connect(self.y_but_minus_func)
        self.z_but_add.clicked.connect(self.yaw_but_add_func)
        self.z_but_minus.clicked.connect(self.yaw_but_minus_func)

        self.front_plate_add_but.clicked.connect(self.front_plate_add_func)
        self.front_plate_minus_but.clicked.connect(self.front_plate_minus_func)
        self.behind_plate_add_but.clicked.connect(self.behind_plate_add_func)
        self.behind_plate_minus_but.clicked.connect(self.behind_plate_minus_func)

        self.brush_add_but.clicked.connect(self.brush_add_but_func)
        self.brush_minus_but.clicked.connect(self.brush_minus_but_func)
        self.fan_add_but.clicked.connect(self.fan_add_but_func)
        self.fan_minus_but.clicked.connect(self.fan_minus_but_func)

        self.servo1_add_but.clicked.connect(self.servo1_add_but_func)
        self.servo2_add_but.clicked.connect(self.servo2_add_but_func)
        self.servo3_add_but.clicked.connect(self.servo3_add_but_func)
        self.servo4_add_but.clicked.connect(self.servo4_add_but_func)
        self.servo5_add_but.clicked.connect(self.servo5_add_but_func)
        self.servo6_add_but.clicked.connect(self.servo6_add_but_func)

        self.servo1_minus_but.clicked.connect(self.servo1_minus_but_func)
        self.servo2_minus_but.clicked.connect(self.servo2_minus_but_func)
        self.servo3_minus_but.clicked.connect(self.servo3_minus_but_func)
        self.servo4_minus_but.clicked.connect(self.servo4_minus_but_func)
        self.servo5_minus_but.clicked.connect(self.servo5_minus_but_func)
        self.servo6_minus_but.clicked.connect(self.servo6_minus_but_func)

        self.send_stm32_vel.clicked.connect(self.send_stm32_vel_func)
        self.stop.clicked.connect(self.stop_func)
        self.send_front_plate.clicked.connect(self.send_front_plate_func)
        self.send_behind_plate.clicked.connect(self.send_behind_plate_func)
        self.send_brush.clicked.connect(self.send_brush_func)
        self.send_fan.clicked.connect(self.send_fan_func)
        self.send_arm.clicked.connect(self.send_arm_func)

        self.change_vel.clicked.connect(self.change_func)
        self.change_vel_text = ["x1","x10","x0.1"]
        self.change_vel_ = [1,10,0.1]
        self.change_vel_index = 0
        self.add_val = self.change_vel_[self.change_vel_index]

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_text)
        self.timer.start(200)

        self.pic_timer = QTimer()
        self.pic_timer.timeout.connect(self.update_pic)
        self.pic_timer.start(50)

        self.lock = False
        self.frame = np.zeros((640,480))
        # self.my_tcp = tcp(is_sender=False)
        self.udp = UDP_Manager(self.cb_leaf_image)
        self.udp.Start()
        self.FLR_ip = rospy.get_param("/FLR_ip")
        self.FLR_port = rospy.get_param("/FLR_port")
        self.udp.targetDict[(self.FLR_ip,self.FLR_port)] = 1
        self.udp.Send(b'req')
        
        # self.bridge = CvBridge()
        # rospy.Subscriber("/leaf_image", Image,self.cb_leaf_image)


    def cb_leaf_image(self,recvData, recvAddr):
        if self.lock == False: 
            data = np.frombuffer(recvData, dtype=np.uint8)
            print(recvData)
            self.frame = cv2.imdecode(data, cv2.IMWRITE_JPEG_QUALITY)
            # self.frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            self.lock = True

    def change_func(self):
        self.change_vel_index += 1
        self.change_vel_index %= len(self.change_vel_text)
        self.change_vel.setText(self.change_vel_text[self.change_vel_index])
        self.add_val = self.change_vel_[self.change_vel_index]

    def update_text(self):
        self.x_text.setText(str(self.vel[0]))
        self.y_text.setText(str(self.vel[1]))
        self.yaw_text.setText(str(self.vel[2]))

        self.x_text_current.setText(str(self._vel[0]))
        self.y_text_current.setText(str(self._vel[1]))
        self.yaw_text_current.setText(str(self._vel[2]))

        self.front_plate_text.setText(str(self.front_plate))
        self.behind_plate_text.setText(str(self.behind_plate))
        self.brush_text.setText(str(self.brush))
        self.fan_text.setText(str(self.fan))

        self.servo1_text.setText(str(self.servo_angle[0]))
        self.servo2_text.setText(str(self.servo_angle[1]))
        self.servo3_text.setText(str(self.servo_angle[2]))
        self.servo4_text.setText(str(self.servo_angle[3]))
        self.servo5_text.setText(str(self.servo_angle[4]))
        self.servo6_text.setText(str(self.servo_angle[5]))

        self.current_servo1_angle.setText(str(self._servo_angle[0]))
        self.current_servo2_angle.setText(str(self._servo_angle[1]))
        self.current_servo3_angle.setText(str(self._servo_angle[2]))
        self.current_servo4_angle.setText(str(self._servo_angle[3]))
        self.current_servo5_angle.setText(str(self._servo_angle[4]))
        self.current_servo6_angle.setText(str(self._servo_angle[5]))


    def send_stm32_vel_func(self):
        info = stm_vel_cmd()
        info.x = self.vel[0]
        info.y = self.vel[1]
        info.yaw = self.vel[2]
        self.send_stm32_vel_topic.publish(info)
    
    def stop_func(self):
        self.switch_mode_topic.publish()

        self.vel = [0,0,0]
        self.send_stm32_vel_func()

        self.brush = 0
        self.send_brush_func()

        self.fan = 0
        self.send_fan_func()

        self.front_plate = 0
        self.behind_plate = 0
        self.send_front_plate_func()
        self.send_behind_plate_func()

    def send_front_plate_func(self):
        info = plc_cmd()
        info.slide_dis = self.front_plate
        info.arm_dis = 0
        self.send_plate_topic.publish(info)
    
    def send_behind_plate_func(self):
        info = plc_cmd()
        info.arm_dis = self.behind_plate
        info.slide_dis = 0
        self.send_plate_topic.publish(info)
    
    def send_brush_func(self):
        info = stm_brush_cmd()
        info.vel = self.brush
        self.send_brush_topic.publish(info)

    def send_fan_func(self):
        info = stm_fan_cmd()
        info.vel = self.fan
        self.send_fan_topic.publish(info)
    
    def send_arm_func(self):
        info = manual()
        info.angle1 = self.servo_angle[0]
        info.angle2 = self.servo_angle[1]
        info.angle3 = self.servo_angle[2]
        info.angle4 = self.servo_angle[3]
        info.angle5 = self.servo_angle[4]
        info.angle6 = self.servo_angle[5]
        self.send_servo_topic.publish(info)



    def x_but_add_func(self):
        self.vel[0] += self.add_val

    def x_but_minus_func(self):
        self.vel[0] -= self.add_val

    def y_but_add_func(self):
        self.vel[1] += self.add_val
    
    def y_but_minus_func(self):
        self.vel[1] -= self.add_val

    def yaw_but_add_func(self):
        self.vel[2] += self.add_val

    def yaw_but_minus_func(self):
        self.vel[2] -= self.add_val
    
    def front_plate_add_func(self):
        self.front_plate += self.add_val
    def front_plate_minus_func(self):
        self.front_plate -= self.add_val
    def behind_plate_add_func(self):
        self.behind_plate += self.add_val
    def behind_plate_minus_func(self):
        self.behind_plate -= self.add_val
    
    def brush_add_but_func(self):
        self.brush += self.add_val
    def brush_minus_but_func(self):
        self.brush -= self.add_val

    def fan_add_but_func(self):
        self.fan += self.add_val
    def fan_minus_but_func(self):
        self.fan -= self.add_val
    
    def servo1_add_but_func(self):
        self.servo_angle[0] += self.add_val
    def servo2_add_but_func(self):
        self.servo_angle[1] += self.add_val
    def servo3_add_but_func(self):
        self.servo_angle[2] += self.add_val
    def servo4_add_but_func(self):
        self.servo_angle[3] += self.add_val
    def servo5_add_but_func(self):
        self.servo_angle[4] += self.add_val
    def servo6_add_but_func(self):
        self.servo_angle[5] += self.add_val
    
    def servo1_minus_but_func(self):
        self.servo_angle[0] -= self.add_val
    def servo2_minus_but_func(self):
        self.servo_angle[1] -= self.add_val
    def servo3_minus_but_func(self):
        self.servo_angle[2] -= self.add_val
    def servo4_minus_but_func(self):
        self.servo_angle[3] -= self.add_val
    def servo5_minus_but_func(self):
        self.servo_angle[4] -= self.add_val
    def servo6_minus_but_func(self):
        self.servo_angle[5] -= self.add_val

    def display_servo_angle(self,msg):
        servo_angle = msg.data
        for i in range(6):
            self._servo_angle[i] = servo_angle[i]
    
    def display_vel(self,msg):
        self._vel[0] = msg.x
        self._vel[1] = msg.y
        self._vel[2] = msg.yaw

    def update_pic(self):
        if self.lock == True:
        # self.cb_leaf_image()
            try:
                img_rows,img_cols,channels = self.frame.shape
            except:
                return
            bytesPerLine = channels * img_cols
            QImg = QImage(self.frame.data,img_cols,img_rows,bytesPerLine,QImage.Format_BGR888)
            self.show_label.setStyleSheet('background-color: rgb(0, 0, 0)')
            self.show_label.setPixmap(QPixmap.fromImage(QImg).scaled(
                self.show_label.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation
            ))
            self.lock = False

    def setupUi(self, CtlWin):
        CtlWin.setObjectName("CtlWin")
        CtlWin.resize(1120, 720)
        self.centralwidget = QtWidgets.QWidget(CtlWin)
        self.centralwidget.setObjectName("centralwidget")
        self.x_text = QtWidgets.QTextEdit(self.centralwidget)
        self.x_text.setGeometry(QtCore.QRect(20, 50, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.x_text.setFont(font)
        self.x_text.setObjectName("x_text")
        self.x_but_add = QtWidgets.QPushButton(self.centralwidget)
        self.x_but_add.setGeometry(QtCore.QRect(100, 10, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.x_but_add.setFont(font)
        self.x_but_add.setObjectName("x_but_add")
        self.x_but_minus = QtWidgets.QPushButton(self.centralwidget)
        self.x_but_minus.setGeometry(QtCore.QRect(100, 50, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.x_but_minus.setFont(font)
        self.x_but_minus.setObjectName("x_but_minus")
        self.y_but_add = QtWidgets.QPushButton(self.centralwidget)
        self.y_but_add.setGeometry(QtCore.QRect(100, 100, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.y_but_add.setFont(font)
        self.y_but_add.setObjectName("y_but_add")
        self.y_but_minus = QtWidgets.QPushButton(self.centralwidget)
        self.y_but_minus.setGeometry(QtCore.QRect(100, 140, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.y_but_minus.setFont(font)
        self.y_but_minus.setObjectName("y_but_minus")
        self.z_but_add = QtWidgets.QPushButton(self.centralwidget)
        self.z_but_add.setGeometry(QtCore.QRect(100, 190, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.z_but_add.setFont(font)
        self.z_but_add.setObjectName("z_but_add")
        self.z_but_minus = QtWidgets.QPushButton(self.centralwidget)
        self.z_but_minus.setGeometry(QtCore.QRect(100, 230, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.z_but_minus.setFont(font)
        self.z_but_minus.setObjectName("z_but_minus")
        self.front_plate_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.front_plate_add_but.setGeometry(QtCore.QRect(260, 10, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.front_plate_add_but.setFont(font)
        self.front_plate_add_but.setObjectName("front_plate_add_but")
        self.front_plate_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.front_plate_minus_but.setGeometry(QtCore.QRect(260, 50, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.front_plate_minus_but.setFont(font)
        self.front_plate_minus_but.setObjectName("front_plate_minus_but")
        self.behind_plate_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.behind_plate_add_but.setGeometry(QtCore.QRect(260, 100, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.behind_plate_add_but.setFont(font)
        self.behind_plate_add_but.setObjectName("behind_plate_add_but")
        self.behind_plate_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.behind_plate_minus_but.setGeometry(QtCore.QRect(260, 140, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.behind_plate_minus_but.setFont(font)
        self.behind_plate_minus_but.setObjectName("behind_plate_minus_but")
        self.y_text = QtWidgets.QTextEdit(self.centralwidget)
        self.y_text.setGeometry(QtCore.QRect(20, 140, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.y_text.setFont(font)
        self.y_text.setObjectName("y_text")
        self.yaw_text = QtWidgets.QTextEdit(self.centralwidget)
        self.yaw_text.setGeometry(QtCore.QRect(20, 230, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.yaw_text.setFont(font)
        self.yaw_text.setObjectName("yaw_text")
        self.front_plate_text = QtWidgets.QTextEdit(self.centralwidget)
        self.front_plate_text.setGeometry(QtCore.QRect(180, 30, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.front_plate_text.setFont(font)
        self.front_plate_text.setObjectName("front_plate_text")
        self.behind_plate_text = QtWidgets.QTextEdit(self.centralwidget)
        self.behind_plate_text.setGeometry(QtCore.QRect(180, 120, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.behind_plate_text.setFont(font)
        self.behind_plate_text.setObjectName("behind_plate_text")
        self.send_stm32_vel = QtWidgets.QPushButton(self.centralwidget)
        self.send_stm32_vel.setGeometry(QtCore.QRect(20, 280, 121, 71))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.send_stm32_vel.setFont(font)
        self.send_stm32_vel.setObjectName("send_stm32_vel")
        self.stop = QtWidgets.QPushButton(self.centralwidget)
        self.stop.setGeometry(QtCore.QRect(360, 570, 81, 61))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.stop.setFont(font)
        self.stop.setObjectName("stop")
        self.send_front_plate = QtWidgets.QPushButton(self.centralwidget)
        self.send_front_plate.setGeometry(QtCore.QRect(330, 20, 101, 51))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.send_front_plate.setFont(font)
        self.send_front_plate.setObjectName("send_front_plate")
        self.send_behind_plate = QtWidgets.QPushButton(self.centralwidget)
        self.send_behind_plate.setGeometry(QtCore.QRect(330, 110, 101, 51))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.send_behind_plate.setFont(font)
        self.send_behind_plate.setObjectName("send_behind_plate")
        self.fan_text = QtWidgets.QTextEdit(self.centralwidget)
        self.fan_text.setGeometry(QtCore.QRect(180, 310, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.fan_text.setFont(font)
        self.fan_text.setObjectName("fan_text")
        self.send_fan = QtWidgets.QPushButton(self.centralwidget)
        self.send_fan.setGeometry(QtCore.QRect(330, 310, 81, 51))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.send_fan.setFont(font)
        self.send_fan.setObjectName("send_fan")
        self.fan_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.fan_add_but.setGeometry(QtCore.QRect(260, 300, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.fan_add_but.setFont(font)
        self.fan_add_but.setObjectName("fan_add_but")
        self.fan_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.fan_minus_but.setGeometry(QtCore.QRect(260, 340, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.fan_minus_but.setFont(font)
        self.fan_minus_but.setObjectName("fan_minus_but")
        self.servo1_text = QtWidgets.QTextEdit(self.centralwidget)
        self.servo1_text.setGeometry(QtCore.QRect(120, 370, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.servo1_text.setFont(font)
        self.servo1_text.setObjectName("servo1_text")
        self.servo2_text = QtWidgets.QTextEdit(self.centralwidget)
        self.servo2_text.setGeometry(QtCore.QRect(120, 460, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.servo2_text.setFont(font)
        self.servo2_text.setObjectName("servo2_text")
        self.servo3_text = QtWidgets.QTextEdit(self.centralwidget)
        self.servo3_text.setGeometry(QtCore.QRect(120, 550, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.servo3_text.setFont(font)
        self.servo3_text.setObjectName("servo3_text")
        self.servo6_text = QtWidgets.QTextEdit(self.centralwidget)
        self.servo6_text.setGeometry(QtCore.QRect(270, 570, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.servo6_text.setFont(font)
        self.servo6_text.setObjectName("servo6_text")
        self.servo4_text = QtWidgets.QTextEdit(self.centralwidget)
        self.servo4_text.setGeometry(QtCore.QRect(270, 390, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.servo4_text.setFont(font)
        self.servo4_text.setObjectName("servo4_text")
        self.servo5_text = QtWidgets.QTextEdit(self.centralwidget)
        self.servo5_text.setGeometry(QtCore.QRect(270, 480, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.servo5_text.setFont(font)
        self.servo5_text.setObjectName("servo5_text")
        self.servo1_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo1_add_but.setGeometry(QtCore.QRect(60, 410, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo1_add_but.setFont(font)
        self.servo1_add_but.setObjectName("servo1_add_but")
        self.servo1_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo1_minus_but.setGeometry(QtCore.QRect(120, 410, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo1_minus_but.setFont(font)
        self.servo1_minus_but.setObjectName("servo1_minus_but")
        self.servo2_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo2_add_but.setGeometry(QtCore.QRect(60, 500, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo2_add_but.setFont(font)
        self.servo2_add_but.setObjectName("servo2_add_but")
        self.servo2_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo2_minus_but.setGeometry(QtCore.QRect(120, 500, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo2_minus_but.setFont(font)
        self.servo2_minus_but.setObjectName("servo2_minus_but")
        self.servo3_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo3_add_but.setGeometry(QtCore.QRect(60, 590, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo3_add_but.setFont(font)
        self.servo3_add_but.setObjectName("servo3_add_but")
        self.servo3_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo3_minus_but.setGeometry(QtCore.QRect(120, 590, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo3_minus_but.setFont(font)
        self.servo3_minus_but.setObjectName("servo3_minus_but")
        self.servo4_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo4_add_but.setGeometry(QtCore.QRect(210, 430, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo4_add_but.setFont(font)
        self.servo4_add_but.setObjectName("servo4_add_but")
        self.servo4_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo4_minus_but.setGeometry(QtCore.QRect(270, 430, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo4_minus_but.setFont(font)
        self.servo4_minus_but.setObjectName("servo4_minus_but")
        self.servo5_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo5_add_but.setGeometry(QtCore.QRect(210, 520, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo5_add_but.setFont(font)
        self.servo5_add_but.setObjectName("servo5_add_but")
        self.servo5_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo5_minus_but.setGeometry(QtCore.QRect(270, 520, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo5_minus_but.setFont(font)
        self.servo5_minus_but.setObjectName("servo5_minus_but")
        self.servo6_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo6_add_but.setGeometry(QtCore.QRect(210, 610, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo6_add_but.setFont(font)
        self.servo6_add_but.setObjectName("servo6_add_but")
        self.servo6_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo6_minus_but.setGeometry(QtCore.QRect(270, 610, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo6_minus_but.setFont(font)
        self.servo6_minus_but.setObjectName("servo6_minus_but")
        self.send_arm = QtWidgets.QPushButton(self.centralwidget)
        self.send_arm.setGeometry(QtCore.QRect(350, 440, 91, 51))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.send_arm.setFont(font)
        self.send_arm.setObjectName("send_arm")
        self.current_servo1_angle = QtWidgets.QTextEdit(self.centralwidget)
        self.current_servo1_angle.setGeometry(QtCore.QRect(50, 370, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.current_servo1_angle.setFont(font)
        self.current_servo1_angle.setObjectName("current_servo1_angle")
        self.current_servo2_angle = QtWidgets.QTextEdit(self.centralwidget)
        self.current_servo2_angle.setGeometry(QtCore.QRect(50, 460, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.current_servo2_angle.setFont(font)
        self.current_servo2_angle.setObjectName("current_servo2_angle")
        self.current_servo3_angle = QtWidgets.QTextEdit(self.centralwidget)
        self.current_servo3_angle.setGeometry(QtCore.QRect(50, 550, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.current_servo3_angle.setFont(font)
        self.current_servo3_angle.setObjectName("current_servo3_angle")
        self.current_servo4_angle = QtWidgets.QTextEdit(self.centralwidget)
        self.current_servo4_angle.setGeometry(QtCore.QRect(200, 390, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.current_servo4_angle.setFont(font)
        self.current_servo4_angle.setObjectName("current_servo4_angle")
        self.current_servo5_angle = QtWidgets.QTextEdit(self.centralwidget)
        self.current_servo5_angle.setGeometry(QtCore.QRect(200, 480, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.current_servo5_angle.setFont(font)
        self.current_servo5_angle.setObjectName("current_servo5_angle")
        self.current_servo6_angle = QtWidgets.QTextEdit(self.centralwidget)
        self.current_servo6_angle.setGeometry(QtCore.QRect(200, 570, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.current_servo6_angle.setFont(font)
        self.current_servo6_angle.setObjectName("current_servo6_angle")
        self.send_brush = QtWidgets.QPushButton(self.centralwidget)
        self.send_brush.setGeometry(QtCore.QRect(330, 220, 71, 51))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.send_brush.setFont(font)
        self.send_brush.setObjectName("send_brush")
        self.brush_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.brush_add_but.setGeometry(QtCore.QRect(260, 200, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.brush_add_but.setFont(font)
        self.brush_add_but.setObjectName("brush_add_but")
        self.brush_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.brush_minus_but.setGeometry(QtCore.QRect(260, 240, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.brush_minus_but.setFont(font)
        self.brush_minus_but.setObjectName("brush_minus_but")
        self.brush_text = QtWidgets.QTextEdit(self.centralwidget)
        self.brush_text.setGeometry(QtCore.QRect(180, 210, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.brush_text.setFont(font)
        self.brush_text.setObjectName("brush_text")
        self.show_label = QtWidgets.QLabel(self.centralwidget)
        self.show_label.setGeometry(QtCore.QRect(450, 10, 640, 480))
        self.show_label.setText("")
        self.show_label.setAlignment(QtCore.Qt.AlignCenter)
        self.show_label.setObjectName("show_label")
        self.x_text_current = QtWidgets.QTextEdit(self.centralwidget)
        self.x_text_current.setGeometry(QtCore.QRect(20, 10, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.x_text_current.setFont(font)
        self.x_text_current.setObjectName("x_text_current")
        self.y_text_current = QtWidgets.QTextEdit(self.centralwidget)
        self.y_text_current.setGeometry(QtCore.QRect(20, 100, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.y_text_current.setFont(font)
        self.y_text_current.setObjectName("y_text_current")
        self.yaw_text_current = QtWidgets.QTextEdit(self.centralwidget)
        self.yaw_text_current.setGeometry(QtCore.QRect(20, 190, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.yaw_text_current.setFont(font)
        self.yaw_text_current.setObjectName("yaw_text_current")
        self.change_vel = QtWidgets.QPushButton(self.centralwidget)
        self.change_vel.setGeometry(QtCore.QRect(380, 610, 81, 61))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.change_vel.setFont(font)
        self.change_vel.setObjectName("change_vel")
        CtlWin.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(CtlWin)
        self.statusbar.setObjectName("statusbar")
        CtlWin.setStatusBar(self.statusbar)

        self.retranslateUi(CtlWin)
        QtCore.QMetaObject.connectSlotsByName(CtlWin)

    def retranslateUi(self, CtlWin):
        _translate = QtCore.QCoreApplication.translate
        CtlWin.setWindowTitle(_translate("CtlWin", "CtlWin"))
        self.x_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.x_but_add.setText(_translate("CtlWin", "增"))
        self.x_but_minus.setText(_translate("CtlWin", "减"))
        self.y_but_add.setText(_translate("CtlWin", "增"))
        self.y_but_minus.setText(_translate("CtlWin", "减"))
        self.z_but_add.setText(_translate("CtlWin", "增"))
        self.z_but_minus.setText(_translate("CtlWin", "减"))
        self.front_plate_add_but.setText(_translate("CtlWin", "增"))
        self.front_plate_minus_but.setText(_translate("CtlWin", "减"))
        self.behind_plate_add_but.setText(_translate("CtlWin", "增"))
        self.behind_plate_minus_but.setText(_translate("CtlWin", "减"))
        self.y_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.yaw_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.front_plate_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.behind_plate_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.send_stm32_vel.setText(_translate("CtlWin", "速度"))
        self.stop.setText(_translate("CtlWin", "停"))
        self.send_front_plate.setText(_translate("CtlWin", "前滑轨"))
        self.send_behind_plate.setText(_translate("CtlWin", "后滑轨"))
        self.fan_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.send_fan.setText(_translate("CtlWin", "风机"))
        self.fan_add_but.setText(_translate("CtlWin", "增"))
        self.fan_minus_but.setText(_translate("CtlWin", "减"))
        self.servo1_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.servo2_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.servo3_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.servo6_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.servo4_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.servo5_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.servo1_add_but.setText(_translate("CtlWin", "增"))
        self.servo1_minus_but.setText(_translate("CtlWin", "减"))
        self.servo2_add_but.setText(_translate("CtlWin", "增"))
        self.servo2_minus_but.setText(_translate("CtlWin", "减"))
        self.servo3_add_but.setText(_translate("CtlWin", "增"))
        self.servo3_minus_but.setText(_translate("CtlWin", "减"))
        self.servo4_add_but.setText(_translate("CtlWin", "增"))
        self.servo4_minus_but.setText(_translate("CtlWin", "减"))
        self.servo5_add_but.setText(_translate("CtlWin", "增"))
        self.servo5_minus_but.setText(_translate("CtlWin", "减"))
        self.servo6_add_but.setText(_translate("CtlWin", "增"))
        self.servo6_minus_but.setText(_translate("CtlWin", "减"))
        self.send_arm.setText(_translate("CtlWin", "机械臂"))
        self.current_servo1_angle.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.current_servo2_angle.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.current_servo3_angle.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.current_servo4_angle.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.current_servo5_angle.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.current_servo6_angle.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.send_brush.setText(_translate("CtlWin", "刷子"))
        self.brush_add_but.setText(_translate("CtlWin", "增"))
        self.brush_minus_but.setText(_translate("CtlWin", "减"))
        self.brush_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.x_text_current.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.y_text_current.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.yaw_text_current.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.change_vel.setText(_translate("CtlWin", "x1"))

def main():
    app = QApplication(sys.argv) # sys.argv即命令行参数
    mainWindow = QMainWindow()
    mainWidget = Ui_CtlWin(mainWindow) #新建一个主界面
    mainWindow.show()	#显示主界面
    exit(app.exec_()) #进入消息循环

if __name__ == '__main__':
    main()