
from sys import argv

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication,QMainWindow

class Ui_CtlWin(QMainWindow):

    def __init__(self):
        super(Ui_CtlWin, self).__init__()
        self.setupUi(self)

    def setupUi(self, CtlWin):
        CtlWin.setObjectName("CtlWin")
        CtlWin.resize(1118, 717)
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
        self.stop.setGeometry(QtCore.QRect(350, 500, 81, 61))
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
        self.send_arm.setGeometry(QtCore.QRect(350, 410, 91, 81))
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
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(450, 10, 640, 480))
        self.label.setText("")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
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
        self.change_vel.setGeometry(QtCore.QRect(350, 570, 81, 61))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.change_vel.setFont(font)
        self.change_vel.setObjectName("change_vel")
        self.arm_num_text = QtWidgets.QTextEdit(self.centralwidget)
        self.arm_num_text.setGeometry(QtCore.QRect(510, 560, 71, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.arm_num_text.setFont(font)
        self.arm_num_text.setObjectName("arm_num_text")
        self.arm_num_add = QtWidgets.QPushButton(self.centralwidget)
        self.arm_num_add.setGeometry(QtCore.QRect(480, 600, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.arm_num_add.setFont(font)
        self.arm_num_add.setObjectName("arm_num_add")
        self.arm_num_minus = QtWidgets.QPushButton(self.centralwidget)
        self.arm_num_minus.setGeometry(QtCore.QRect(540, 600, 61, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.arm_num_minus.setFont(font)
        self.arm_num_minus.setObjectName("arm_num_minus")
        self.arm_num_but = QtWidgets.QPushButton(self.centralwidget)
        self.arm_num_but.setGeometry(QtCore.QRect(630, 580, 91, 51))
        font = QtGui.QFont()
        font.setPointSize(20)
        self.arm_num_but.setFont(font)
        self.arm_num_but.setObjectName("arm_num_but")
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
        self.x_but_add.setText(_translate("CtlWin", "???"))
        self.x_but_minus.setText(_translate("CtlWin", "???"))
        self.y_but_add.setText(_translate("CtlWin", "???"))
        self.y_but_minus.setText(_translate("CtlWin", "???"))
        self.z_but_add.setText(_translate("CtlWin", "???"))
        self.z_but_minus.setText(_translate("CtlWin", "???"))
        self.front_plate_add_but.setText(_translate("CtlWin", "???"))
        self.front_plate_minus_but.setText(_translate("CtlWin", "???"))
        self.behind_plate_add_but.setText(_translate("CtlWin", "???"))
        self.behind_plate_minus_but.setText(_translate("CtlWin", "???"))
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
        self.send_stm32_vel.setText(_translate("CtlWin", "??????"))
        self.stop.setText(_translate("CtlWin", "???"))
        self.send_front_plate.setText(_translate("CtlWin", "?????????"))
        self.send_behind_plate.setText(_translate("CtlWin", "?????????"))
        self.fan_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.send_fan.setText(_translate("CtlWin", "??????"))
        self.fan_add_but.setText(_translate("CtlWin", "???"))
        self.fan_minus_but.setText(_translate("CtlWin", "???"))
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
        self.servo1_add_but.setText(_translate("CtlWin", "???"))
        self.servo1_minus_but.setText(_translate("CtlWin", "???"))
        self.servo2_add_but.setText(_translate("CtlWin", "???"))
        self.servo2_minus_but.setText(_translate("CtlWin", "???"))
        self.servo3_add_but.setText(_translate("CtlWin", "???"))
        self.servo3_minus_but.setText(_translate("CtlWin", "???"))
        self.servo4_add_but.setText(_translate("CtlWin", "???"))
        self.servo4_minus_but.setText(_translate("CtlWin", "???"))
        self.servo5_add_but.setText(_translate("CtlWin", "???"))
        self.servo5_minus_but.setText(_translate("CtlWin", "???"))
        self.servo6_add_but.setText(_translate("CtlWin", "???"))
        self.servo6_minus_but.setText(_translate("CtlWin", "???"))
        self.send_arm.setText(_translate("CtlWin", "?????????"))
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
        self.send_brush.setText(_translate("CtlWin", "??????"))
        self.brush_add_but.setText(_translate("CtlWin", "???"))
        self.brush_minus_but.setText(_translate("CtlWin", "???"))
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
        self.arm_num_text.setHtml(_translate("CtlWin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:15pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.arm_num_add.setText(_translate("CtlWin", "???"))
        self.arm_num_minus.setText(_translate("CtlWin", "???"))
        self.arm_num_but.setText(_translate("CtlWin", "?????????"))


def main():
    app = QApplication(argv) # sys.argv??????????????????
    mainWindow = Ui_CtlWin() #?????????????????????
    mainWindow.show()	#???????????????
    exit(app.exec_()) #??????????????????

if __name__ == '__main__':
    main()