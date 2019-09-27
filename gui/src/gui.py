#!/usr/bin/env python

# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'car.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#

from PyQt5 import QtCore, QtGui, QtWidgets
import rospy # to handel ros node 
from std_msgs.msg import String
from std_msgs.msg import Byte
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from sensors.msg import x_state
global mdriver, massistant, mselfdriving, speedslow, speedmedium, speedfast,data
mdriver = None
massistant = None
mselfdriving = None
speedslow = None
speedmedium = None
speedfast = None 
data=None

x_state1=x_state()
rospy.init_node('gui')

pub_video_stream = rospy.Publisher('video_stream', Bool, queue_size=10)
pub_mode = rospy.Publisher('mode', Byte,queue_size=10)
pub_cruise = rospy.Publisher('cruise_control_speed',Float32,queue_size=10)
pub_emerg = rospy.Publisher('emergency', Bool,queue_size=10)
pub_cruise_dist = rospy.Publisher('cruise_control_min_distance', Byte,queue_size=10)
rate=rospy.Rate(1)
class Ui_Form(object):


    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(800, 480)
	self.pushButton = QtWidgets.QPushButton(Form)
	self.pushButton.setGeometry(QtCore.QRect(440, 150, 121, 25))
	self.pushButton.setObjectName("pushButton")

        self.rbdriver = QtWidgets.QRadioButton(Form)
        self.rbdriver.setGeometry(QtCore.QRect(180, 32, 111, 21))
        self.rbdriver.setObjectName("rbdriver")
        self.rbassistant = QtWidgets.QRadioButton(Form)
        self.rbassistant.setGeometry(QtCore.QRect(420, 30, 112, 23))
        self.rbassistant.setObjectName("rbassistant")
        self.rbselfdriving = QtWidgets.QRadioButton(Form)
        self.rbselfdriving.setGeometry(QtCore.QRect(640, 30, 112, 23))
        self.rbselfdriving.setObjectName("rbselfdriving")
        self.lmincd = QtWidgets.QLabel(Form)
        self.lmincd.setGeometry(QtCore.QRect(20, 250, 131, 17))
        self.lmincd.setObjectName("lmincd")
        self.emergency = QtWidgets.QPushButton(Form)
        self.emergency.setGeometry(QtCore.QRect(30, 340, 89, 25))
        self.emergency.setObjectName("emergency")
        self.rbslow = QtWidgets.QRadioButton(Form)
        self.rbslow.setGeometry(QtCore.QRect(210, 290, 112, 23))
        self.rbslow.setObjectName("rbslow")
        self.rbmedium = QtWidgets.QRadioButton(Form)
        self.rbmedium.setGeometry(QtCore.QRect(410, 290, 112, 23))
        self.rbmedium.setObjectName("rbmedium")
        self.rbfast = QtWidgets.QRadioButton(Form)
        self.rbfast.setGeometry(QtCore.QRect(640, 290, 112, 23))
        self.rbfast.setObjectName("rbfast")
        self.spinBox = QtWidgets.QSpinBox(Form)
        self.spinBox.setGeometry(QtCore.QRect(210, 240, 48, 26))
        self.spinBox.setObjectName("spinBox")
        self.ldeparture = QtWidgets.QLabel(Form)
        self.ldeparture.setGeometry(QtCore.QRect(40, 110, 71, 17))
        self.ldeparture.setObjectName("ldeparture")
        self.lmode = QtWidgets.QLabel(Form)
        self.lmode.setGeometry(QtCore.QRect(40, 40, 67, 17))
        self.lmode.setObjectName("lmode")
        self.lopjpos = QtWidgets.QLabel(Form)
        self.lopjpos.setGeometry(QtCore.QRect(40, 150, 101, 17))
        self.lopjpos.setObjectName("lopjpos")
        self.lcv = QtWidgets.QLabel(Form)
        self.lcv.setGeometry(QtCore.QRect(40, 200, 101, 17))
        self.lcv.setObjectName("lcv")
        self.lccs = QtWidgets.QLabel(Form)
        self.lccs.setGeometry(QtCore.QRect(20, 300, 141, 20))
        self.lccs.setObjectName("lccs")
        self.lr_departure = QtWidgets.QLabel(Form)
        self.lr_departure.setGeometry(QtCore.QRect(250, 110, 67, 17))
        self.lr_departure.setObjectName("lr_departure")
        self.lr_objpos = QtWidgets.QLabel(Form)
        self.lr_objpos.setGeometry(QtCore.QRect(250, 150, 67, 17))
        self.lr_objpos.setObjectName("lr_objpos")
        self.label_8 = QtWidgets.QLabel(Form)
        self.label_8.setGeometry(QtCore.QRect(250, 200, 67, 17))
        self.label_8.setObjectName("label_8")
        self.label = QtWidgets.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(340, 240, 67, 17))
        self.label.setObjectName("label")

	self.pushButton.clicked.connect(self.showit)
        self.retranslateUi(Form)
        self.rbdriver.clicked.connect(self.driver)
        self.rbassistant.clicked.connect(self.assistant)
        self.rbselfdriving.clicked.connect(self.selfdriving)
        self.rbslow.clicked.connect(self.slow)
        self.rbmedium.clicked.connect(self.medium)
        self.rbfast.clicked.connect(self.fast)
        self.emergency.clicked.connect(self.emerg)
        self.spinBox.valueChanged['int'].connect(self.label.setNum)
        self.spinBox.valueChanged['int'].connect(self.mmincd)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
	x_state1=x_state()
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))

        self.pushButton.setText(_translate("Form", "lane detection"))

        self.rbdriver.setText(_translate("Form", "&driver"))
        self.rbassistant.setText(_translate("Form", "assistant"))
        self.rbselfdriving.setText(_translate("Form", "self drivin&g"))
        self.lmincd.setText(_translate("Form", "min cruise distance"))
        self.emergency.setText(_translate("Form", "emergency"))
        self.rbslow.setText(_translate("Form", "s&low"))
        self.rbmedium.setText(_translate("Form", "medi&um"))
        self.rbfast.setText(_translate("Form", "fas&t"))
        self.ldeparture.setText(_translate("Form", "departure"))
        self.lmode.setText(_translate("Form", "mode"))
        self.lopjpos.setText(_translate("Form", "object position"))
        self.lcv.setText(_translate("Form", "curise velocity"))
        self.lccs.setText(_translate("Form", "cruise control speed"))
        self.lr_departure.setText(_translate("Form", "No Data"))
        self.lr_objpos.setText(_translate("Form", "No Data"))
        self.label_8.setText(_translate("Form", "No Data"))
        self.label.setText(_translate("Form", "0"))

	self.cruise_velocity_Callback(x_state1)
	self.cruise_velocity_listener()
	
	self.departure_Callback(data)
	self.departure_listener()
    def departure_Callback(self,data):
	text_sub=str(data)
	self.lr_departure.setText(text_sub)

    
    def departure_listener(self):
	rospy.Subscriber('image_lane_departure', String, self.departure_Callback)
    
    def cruise_velocity_Callback(self,x_state1):
	text_sub=str(x_state1.vx)
	text_sub2=str(x_state1.px)
	self.label_8.setText(text_sub)
	self.lr_objpos.setText(text_sub2)
	print text_sub2
    
    
    def cruise_velocity_listener(self):
	sub=rospy.Subscriber('object_position', x_state, self.cruise_velocity_Callback)


    def showit(self):
    	print "lane_detection"
	pub_video_stream.publish(1)


    def driver(self):

	pub_mode.publish(3)
       # global m_driver
        #m_driver = 3
        print "driving mode 3"
	 
    def assistant(self):
	pub_mode.publish(2)
        #global m_assistant
        #m_assistant = 2
	print "driving assistant 2"
    def selfdriving(self):
	pub_mode.publish(1)
        #global m_selfdriving
        #m_selfdriving = 1
        print " self driving 1"
    def slow(self):
	pub_cruise.publish(4)

        print "cruise_control_speed 4 sec"
    def medium(self):
        pub_cruise.publish(2.5)

        print "cruise_control_speed 2.5 sec"
    def fast(self):
        pub_cruise.publish(1.6)

        print "cruise_control_speed 1.6 sec"

    def emerg(self):
	pub_emerg.publish(1)
        print "emergecy"
    def mmincd(self):
        x = self.spinBox.value()
	pub_cruise_dist.publish(x)
        print "cruise_control_min_distance"
	print x

    rate.sleep()

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())

