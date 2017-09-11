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

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

degToRad = 3.14/180
fingerDip = 30.0 * degToRad

thumbDip = 30.0 * degToRad
thumbMcp = 30.0 * degToRad

factor = 1.2


class handJointController(QWidget, QThread):
    def __init__(self):
        super(self.__class__, self).__init__()

        self.initROS()

        self.initUI()

        self.inc = 0
        self.jointMcp = 10.0 *degToRad
        self.thumbCarpal = 0.0
        self.thumbPip = 0.0

        self.thumbCarpalUp()
        self.thumbPipClosed()

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
            joint_state.position = [self.jointMcp, factor * self.jointMcp, fingerDip,
                                    self.jointMcp, factor * self.jointMcp, fingerDip,
                                    self.jointMcp, factor * self.jointMcp, fingerDip,
                                    self.jointMcp, factor * self.jointMcp, fingerDip,
                                    -self.thumbCarpal, thumbMcp, self.thumbPip,  thumbDip]
            joint_state.velocity = []
            joint_state.effort = []

            self.pub.publish(joint_state)
        else:
            print("ROS is shutdown")

    def closeEvent(self, event):
        self.timer.stop()

    def fingerValue(self):
        self.jointMcp = float(self.fingerMcp_slider.value()) * degToRad

    def thumbCarpalUp(self):
        self.thumbCarpal = 78.0 * degToRad

    def thumbCarpalDown(self):
        self.thumbCarpal = 20.0 * degToRad

    def thumbPipOpen(self):
        self.thumbPip = -10.0 * degToRad

    def thumbPipClosed(self):
        self.thumbPip = 25.0 * degToRad


    def initROS(self):
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        rospy.init_node('joint_state_publisher', anonymous=True)

    def initUI(self):
        fingerMcp = QLabel('Finger MCP Joint')
        self.fingerMcp_slider = QSlider(Qt.Horizontal)
        self.fingerMcp_slider.setMinimum(00)
        self.fingerMcp_slider.setMaximum(90)
        self.fingerMcp_slider.setValue(10)
        self.fingerMcp_slider.setTickPosition(QSlider.TicksBelow)
        self.fingerMcp_slider.setTickInterval(5)

        self.fingerMcp_slider.valueChanged.connect(self.fingerValue)

        thumbCarpalUp = QPushButton('Thumb Up')
        thumbCarpalUp.clicked.connect(self.thumbCarpalUp)

        thumbCarpalDown = QPushButton('Thumb Down')
        thumbCarpalDown.clicked.connect(self.thumbCarpalDown)

        thumbPipOpen = QPushButton('Thumb Open')
        thumbPipOpen.clicked.connect(self.thumbPipOpen)

        thumbPipClosed = QPushButton('Thumb Close')
        thumbPipClosed.clicked.connect(self.thumbPipClosed)

        grid = QGridLayout()
        grid.setSpacing(10)

        grid.addWidget(fingerMcp, 1, 0)
        grid.addWidget(self.fingerMcp_slider, 1, 1)
        grid.addWidget(thumbCarpalUp, 2, 0)
        grid.addWidget(thumbCarpalDown, 2, 1)
        grid.addWidget(thumbPipOpen, 3, 0)
        grid.addWidget(thumbPipClosed, 3, 1)

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