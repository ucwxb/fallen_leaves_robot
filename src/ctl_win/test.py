# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'untitled.ui'
#
# Created by: PyQt5 UI code generator 5.15.3
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_CtlWin(object):
    def setupUi(self, CtlWin):
        CtlWin.setObjectName("CtlWin")
        CtlWin.resize(800, 619)
        self.centralwidget = QtWidgets.QWidget(CtlWin)
        self.centralwidget.setObjectName("centralwidget")
        self.x_text = QtWidgets.QTextEdit(self.centralwidget)
        self.x_text.setGeometry(QtCore.QRect(20, 40, 71, 41))
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
        self.x_but_minus.setGeometry(QtCore.QRect(100, 60, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.x_but_minus.setFont(font)
        self.x_but_minus.setObjectName("x_but_minus")
        self.y_but_add = QtWidgets.QPushButton(self.centralwidget)
        self.y_but_add.setGeometry(QtCore.QRect(100, 130, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.y_but_add.setFont(font)
        self.y_but_add.setObjectName("y_but_add")
        self.y_but_minus = QtWidgets.QPushButton(self.centralwidget)
        self.y_but_minus.setGeometry(QtCore.QRect(100, 180, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.y_but_minus.setFont(font)
        self.y_but_minus.setObjectName("y_but_minus")
        self.z_but_add = QtWidgets.QPushButton(self.centralwidget)
        self.z_but_add.setGeometry(QtCore.QRect(100, 240, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.z_but_add.setFont(font)
        self.z_but_add.setObjectName("z_but_add")
        self.z_but_minus = QtWidgets.QPushButton(self.centralwidget)
        self.z_but_minus.setGeometry(QtCore.QRect(100, 290, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.z_but_minus.setFont(font)
        self.z_but_minus.setObjectName("z_but_minus")
        self.front_plate_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.front_plate_add_but.setGeometry(QtCore.QRect(310, 10, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.front_plate_add_but.setFont(font)
        self.front_plate_add_but.setObjectName("front_plate_add_but")
        self.front_plate_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.front_plate_minus_but.setGeometry(QtCore.QRect(310, 60, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.front_plate_minus_but.setFont(font)
        self.front_plate_minus_but.setObjectName("front_plate_minus_but")
        self.behind_plate_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.behind_plate_add_but.setGeometry(QtCore.QRect(310, 120, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.behind_plate_add_but.setFont(font)
        self.behind_plate_add_but.setObjectName("behind_plate_add_but")
        self.behind_plate_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.behind_plate_minus_but.setGeometry(QtCore.QRect(310, 180, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.behind_plate_minus_but.setFont(font)
        self.behind_plate_minus_but.setObjectName("behind_plate_minus_but")
        self.y_text = QtWidgets.QTextEdit(self.centralwidget)
        self.y_text.setGeometry(QtCore.QRect(20, 160, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.y_text.setFont(font)
        self.y_text.setObjectName("y_text")
        self.yaw_text = QtWidgets.QTextEdit(self.centralwidget)
        self.yaw_text.setGeometry(QtCore.QRect(20, 270, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.yaw_text.setFont(font)
        self.yaw_text.setObjectName("yaw_text")
        self.front_plate_text = QtWidgets.QTextEdit(self.centralwidget)
        self.front_plate_text.setGeometry(QtCore.QRect(230, 30, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.front_plate_text.setFont(font)
        self.front_plate_text.setObjectName("front_plate_text")
        self.behind_plate_text = QtWidgets.QTextEdit(self.centralwidget)
        self.behind_plate_text.setGeometry(QtCore.QRect(230, 150, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.behind_plate_text.setFont(font)
        self.behind_plate_text.setObjectName("behind_plate_text")
        self.send_stm32_vel = QtWidgets.QPushButton(self.centralwidget)
        self.send_stm32_vel.setGeometry(QtCore.QRect(40, 350, 121, 71))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.send_stm32_vel.setFont(font)
        self.send_stm32_vel.setObjectName("send_stm32_vel")
        self.stop = QtWidgets.QPushButton(self.centralwidget)
        self.stop.setGeometry(QtCore.QRect(40, 430, 121, 71))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.stop.setFont(font)
        self.stop.setObjectName("stop")
        self.send_front_plate = QtWidgets.QPushButton(self.centralwidget)
        self.send_front_plate.setGeometry(QtCore.QRect(380, 30, 161, 51))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.send_front_plate.setFont(font)
        self.send_front_plate.setObjectName("send_front_plate")
        self.send_behind_plate = QtWidgets.QPushButton(self.centralwidget)
        self.send_behind_plate.setGeometry(QtCore.QRect(380, 140, 161, 51))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.send_behind_plate.setFont(font)
        self.send_behind_plate.setObjectName("send_behind_plate")
        self.fan_text = QtWidgets.QTextEdit(self.centralwidget)
        self.fan_text.setGeometry(QtCore.QRect(230, 310, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.fan_text.setFont(font)
        self.fan_text.setObjectName("fan_text")
        self.send_fan = QtWidgets.QPushButton(self.centralwidget)
        self.send_fan.setGeometry(QtCore.QRect(380, 300, 161, 51))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.send_fan.setFont(font)
        self.send_fan.setObjectName("send_fan")
        self.fan_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.fan_add_but.setGeometry(QtCore.QRect(310, 270, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.fan_add_but.setFont(font)
        self.fan_add_but.setObjectName("fan_add_but")
        self.fan_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.fan_minus_but.setGeometry(QtCore.QRect(310, 340, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.fan_minus_but.setFont(font)
        self.fan_minus_but.setObjectName("fan_minus_but")
        self.servo1_text = QtWidgets.QTextEdit(self.centralwidget)
        self.servo1_text.setGeometry(QtCore.QRect(220, 390, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.servo1_text.setFont(font)
        self.servo1_text.setObjectName("servo1_text")
        self.servo2_text = QtWidgets.QTextEdit(self.centralwidget)
        self.servo2_text.setGeometry(QtCore.QRect(350, 390, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.servo2_text.setFont(font)
        self.servo2_text.setObjectName("servo2_text")
        self.servo3_text = QtWidgets.QTextEdit(self.centralwidget)
        self.servo3_text.setGeometry(QtCore.QRect(500, 390, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.servo3_text.setFont(font)
        self.servo3_text.setObjectName("servo3_text")
        self.servo6_text = QtWidgets.QTextEdit(self.centralwidget)
        self.servo6_text.setGeometry(QtCore.QRect(360, 490, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.servo6_text.setFont(font)
        self.servo6_text.setObjectName("servo6_text")
        self.servo4_text = QtWidgets.QTextEdit(self.centralwidget)
        self.servo4_text.setGeometry(QtCore.QRect(640, 390, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.servo4_text.setFont(font)
        self.servo4_text.setObjectName("servo4_text")
        self.servo5_text = QtWidgets.QTextEdit(self.centralwidget)
        self.servo5_text.setGeometry(QtCore.QRect(210, 490, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.servo5_text.setFont(font)
        self.servo5_text.setObjectName("servo5_text")
        self.servo1_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo1_add_but.setGeometry(QtCore.QRect(190, 440, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo1_add_but.setFont(font)
        self.servo1_add_but.setObjectName("servo1_add_but")
        self.servo1_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo1_minus_but.setGeometry(QtCore.QRect(260, 440, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo1_minus_but.setFont(font)
        self.servo1_minus_but.setObjectName("servo1_minus_but")
        self.servo2_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo2_add_but.setGeometry(QtCore.QRect(330, 440, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo2_add_but.setFont(font)
        self.servo2_add_but.setObjectName("servo2_add_but")
        self.servo2_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo2_minus_but.setGeometry(QtCore.QRect(400, 440, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo2_minus_but.setFont(font)
        self.servo2_minus_but.setObjectName("servo2_minus_but")
        self.servo3_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo3_add_but.setGeometry(QtCore.QRect(470, 440, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo3_add_but.setFont(font)
        self.servo3_add_but.setObjectName("servo3_add_but")
        self.servo3_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo3_minus_but.setGeometry(QtCore.QRect(540, 440, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo3_minus_but.setFont(font)
        self.servo3_minus_but.setObjectName("servo3_minus_but")
        self.servo4_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo4_add_but.setGeometry(QtCore.QRect(610, 440, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo4_add_but.setFont(font)
        self.servo4_add_but.setObjectName("servo4_add_but")
        self.servo4_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo4_minus_but.setGeometry(QtCore.QRect(680, 440, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo4_minus_but.setFont(font)
        self.servo4_minus_but.setObjectName("servo4_minus_but")
        self.servo5_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo5_add_but.setGeometry(QtCore.QRect(170, 540, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo5_add_but.setFont(font)
        self.servo5_add_but.setObjectName("servo5_add_but")
        self.servo5_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo5_minus_but.setGeometry(QtCore.QRect(240, 540, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo5_minus_but.setFont(font)
        self.servo5_minus_but.setObjectName("servo5_minus_but")
        self.servo6_add_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo6_add_but.setGeometry(QtCore.QRect(330, 540, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo6_add_but.setFont(font)
        self.servo6_add_but.setObjectName("servo6_add_but")
        self.servo6_minus_but = QtWidgets.QPushButton(self.centralwidget)
        self.servo6_minus_but.setGeometry(QtCore.QRect(400, 540, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.servo6_minus_but.setFont(font)
        self.servo6_minus_but.setObjectName("servo6_minus_but")
        self.send_arm = QtWidgets.QPushButton(self.centralwidget)
        self.send_arm.setGeometry(QtCore.QRect(510, 500, 161, 51))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.send_arm.setFont(font)
        self.send_arm.setObjectName("send_arm")
        self.current_servo1_angle = QtWidgets.QLabel(self.centralwidget)
        self.current_servo1_angle.setGeometry(QtCore.QRect(620, 30, 111, 41))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.current_servo1_angle.setFont(font)
        self.current_servo1_angle.setText("")
        self.current_servo1_angle.setObjectName("current_servo1_angle")
        self.current_servo2_angle = QtWidgets.QLabel(self.centralwidget)
        self.current_servo2_angle.setGeometry(QtCore.QRect(620, 80, 111, 41))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.current_servo2_angle.setFont(font)
        self.current_servo2_angle.setText("")
        self.current_servo2_angle.setObjectName("current_servo2_angle")
        self.current_servo3_angle = QtWidgets.QLabel(self.centralwidget)
        self.current_servo3_angle.setGeometry(QtCore.QRect(620, 130, 111, 41))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.current_servo3_angle.setFont(font)
        self.current_servo3_angle.setText("")
        self.current_servo3_angle.setObjectName("current_servo3_angle")
        self.current_servo4_angle = QtWidgets.QLabel(self.centralwidget)
        self.current_servo4_angle.setGeometry(QtCore.QRect(620, 170, 111, 41))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.current_servo4_angle.setFont(font)
        self.current_servo4_angle.setText("")
        self.current_servo4_angle.setObjectName("current_servo4_angle")
        self.current_servo5_angle = QtWidgets.QLabel(self.centralwidget)
        self.current_servo5_angle.setGeometry(QtCore.QRect(620, 210, 111, 41))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.current_servo5_angle.setFont(font)
        self.current_servo5_angle.setText("")
        self.current_servo5_angle.setObjectName("current_servo5_angle")
        self.current_servo6_angle = QtWidgets.QLabel(self.centralwidget)
        self.current_servo6_angle.setGeometry(QtCore.QRect(620, 260, 111, 41))
        font = QtGui.QFont()
        font.setPointSize(18)
        self.current_servo6_angle.setFont(font)
        self.current_servo6_angle.setText("")
        self.current_servo6_angle.setObjectName("current_servo6_angle")
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
        self.send_stm32_vel.setText(_translate("CtlWin", "发送速度"))
        self.stop.setText(_translate("CtlWin", "停止"))
        self.send_front_plate.setText(_translate("CtlWin", "发送前滑轨"))
        self.send_behind_plate.setText(_translate("CtlWin", "发送后滑轨"))
        self.fan_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.send_fan.setText(_translate("CtlWin", "发送风机"))
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
        self.send_arm.setText(_translate("CtlWin", "发送机械臂"))
