#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
BionicoHand project

A simple QtGui to control the joint position
of an URDF hand model through ROS message "joint_states"

author: David Gouaillier
last edited: Aout 2017
"""

import sys
import numpy as np
from pylab import *

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

degToRad = 3.14/180
fingerDip = 25.0 * degToRad
fingerPip = 30.0 * degToRad

MiddleDip = 25.0 * degToRad
MiddlePip = 30.0 * degToRad
MiddleMcp =  0.0 * degToRad

thumbDip = 20.0 * degToRad
thumbPip = 30.0 * degToRad

thumbCarpalMin  = -19.0 * degToRad
# thumbCarpalMean = -50.0 * degToRad
thumbCarpalMax  = -80.0 * degToRad

X_FINGER = False
X_FINGER_PIP = False


class handJointController(QWidget, QThread):
    def __init__(self):
        super(self.__class__, self).__init__()

        self.initROS()

        self.initUI()

        self.inc = 0

        self.fingerMcp = 0.0
        self.thumbCarpal = 0.0
        self.thumbMcp = 0.0

        self.motorValue()
        self.thumbCarpalValue()

        self.timer = QTimer()
        self.timer.timeout.connect(self.publishJointState)
        self.timer.start(100) #10 Hz

    def publishJointState(self):
        if not rospy.is_shutdown():
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = ['IndexMcp'   , 'IndexPip' , 'IndexDip' ,
                                'MiddleMcp'  , 'MiddlePip', 'MiddleDip',
                                'RingMcp'    , 'RingPip'  , 'RingDip'  ,
                                'PinkyMcp'   , 'PinkyPip' , 'PinkyDip' ,
                                'ThumbCarpal', 'ThumbMcp' , 'ThumbPip' , 'ThumbDip'
                                ]

            if(X_FINGER):
                joint_state.position = [self.fingerMcp, self.fingerMcp, self.fingerMcp,
                                        self.fingerMcp, self.fingerMcp, self.fingerMcp,
                                        self.fingerMcp, self.fingerMcp, self.fingerMcp,
                                        self.fingerMcp, self.fingerMcp, self.fingerMcp,
                                        self.thumbCarpal, self.thumbMcp, self.thumbMcp, self.thumbMcp]
            elif(X_FINGER_PIP):
                joint_state.position = [self.fingerMcp, self.fingerMcp, fingerDip,
                                        self.fingerMcp, self.fingerMcp, MiddleDip,
                                        self.fingerMcp, self.fingerMcp, fingerDip,
                                        self.fingerMcp, self.fingerMcp, fingerDip,
                                        self.thumbCarpal, self.thumbMcp, self.thumbMcp, thumbDip]
            else:
                joint_state.position = [self.fingerMcp, fingerPip, fingerDip,
                                        self.fingerMcp, MiddlePip, MiddleDip,
                                        self.fingerMcp, fingerPip, fingerDip,
                                        self.fingerMcp, fingerPip, fingerDip,
                                        self.thumbCarpal, self.thumbMcp, thumbPip, thumbDip]

            joint_state.velocity = []
            joint_state.effort = []

            self.pub.publish(joint_state)
        else:
            print("ROS is shutdown")

    def closeEvent(self, event):
        self.timer.stop()

    def motorValue(self):
        sliderVal = float(self.motor_slider.value())

        fingerVal         = [  60.0,  60.0,   5.0,  5.0, 90.0]
        motorForFingerVal = [-100.0, -50.0, -20.0, 20.0, 100.0]
        self.fingerMcp = np.interp(sliderVal, motorForFingerVal, fingerVal)
        print sliderVal," --> ",self.fingerMcp

        thumbVal         = [  45.0, 15.0,  38.0,  0.0, 30.0,  30.0]
        motorForThumbVal = [-100.0,-50.0, -20.0, 20.0, 50.0, 100.0]
        self.thumbMcp = np.interp(sliderVal, motorForThumbVal, thumbVal)

        # CONVERT to Radian
        self.fingerMcp *= degToRad
        self.thumbMcp  *= degToRad

        # figure()
        # subplot(2,1,1)
        # plot(motorForFingerVal, fingerVal, 'r')
        # subplot(2,1,2)
        # plot(motorForThumbVal, thumbVal, 'k')
        # show()
        # exit()

    def thumbCarpalValue(self):
        sliderVal = int(self.thumbCarpal_slider.value())
        if(sliderVal == 0):
            self.thumbCarpal = thumbCarpalMin
        # elif(sliderVal == 1):
        #     self.thumbCarpal = thumbCarpalMean
        else:
            self.thumbCarpal = thumbCarpalMax



    def initROS(self):
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        rospy.init_node('joint_state_publisher', anonymous=True)

    def initUI(self):
        motorValue = QLabel('Motor Value [linear Screw]')
        self.motor_slider = QSlider(Qt.Vertical)
        self.motor_slider.setRange(-100.0, 100.0)
        self.motor_slider.setValue(0.0)
        self.motor_slider.setTickPosition(QSlider.TicksBelow)
        self.motor_slider.setTickInterval(50)

        self.motor_slider.valueChanged.connect(self.motorValue)

        thumbCarpal = QLabel('Thumb Carpal Joint')
        self.thumbCarpal_slider = QSlider(Qt.Horizontal)
        self.thumbCarpal_slider.setRange(0,1)
        self.thumbCarpal_slider.setValue(0)
        self.thumbCarpal_slider.setTickPosition(QSlider.TicksBelow)
        self.thumbCarpal_slider.setTickInterval(1)

        self.thumbCarpal_slider.valueChanged.connect(self.thumbCarpalValue)


        grid = QGridLayout()
        grid.setSpacing(10)

        grid.addWidget(motorValue, 1, 0)
        grid.addWidget(self.motor_slider, 1, 1)
        grid.addWidget(thumbCarpal, 2, 0)
        grid.addWidget(self.thumbCarpal_slider, 3, 0)

        self.setLayout(grid)

        self.setGeometry(300, 300, 350, 300)
        self.setWindowTitle('hand joint controller')
        self.show()


if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)
        ex = handJointController()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass