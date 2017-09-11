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
fingerDip = 15.0 * degToRad


class handJointController(QWidget, QThread):
    def __init__(self):
        super(self.__class__, self).__init__()

        self.initROS()

        self.initUI()

        self.inc = 0
        self.jointMcp = 0.0
        self.thumbCarpal = 0.0
        self.thumbMcp = 0.0

        self.timer = QTimer()
        self.timer.timeout.connect(self.publishJointState)
        self.timer.start(100) #10 Hz

    def publishJointState(self):
        if not rospy.is_shutdown():
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = ['indexMcp'   , 'indexPip' , 'indexDip' ,
                                'middleMcp'  , 'middlePip', 'middleDip',
                                'ringMcp'    , 'ringPip'  , 'ringDip'  ,
                                'pinkyMcp'   , 'pinkyPip' , 'pinkyDip' ,
                                'thumbCarpal', 'thumbMcp' , 'thumbPip' , 'thumbDip'
                                ]
            joint_state.position = [self.jointMcp, self.jointMcp, fingerDip,
                                    self.jointMcp, self.jointMcp, fingerDip,
                                    self.jointMcp, self.jointMcp, fingerDip,
                                    self.jointMcp, self.jointMcp, fingerDip,
                                    -self.thumbCarpal, self.thumbMcp, self.thumbMcp, fingerDip]
            joint_state.velocity = []
            joint_state.effort = []

            self.pub.publish(joint_state)
        else:
            print("ROS is shutdown")

    def closeEvent(self, event):
        self.timer.stop()

    def fingerValue(self):
        self.jointMcp = float(self.fingerMcp_slider.value()) * degToRad

    def thumbCarpalValue(self):
        self.thumbCarpal = float(self.thumbCarpal_slider.value()) * degToRad

    def thumbMcpValue(self):
        self.thumbMcp = float(self.thumbMcp_slider.value()) * degToRad

    def initROS(self):
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        rospy.init_node('joint_state_publisher', anonymous=True)

    def initUI(self):
        fingerMcp = QLabel('Finger MCP Joint')
        self.fingerMcp_slider = QSlider(Qt.Horizontal)
        self.fingerMcp_slider.setMinimum(00)
        self.fingerMcp_slider.setMaximum(90)
        self.fingerMcp_slider.setValue(0)
        self.fingerMcp_slider.setTickPosition(QSlider.TicksBelow)
        self.fingerMcp_slider.setTickInterval(5)

        self.fingerMcp_slider.valueChanged.connect(self.fingerValue)

        thumbCarpal = QLabel('Thumb Carpal Joint')
        self.thumbCarpal_slider = QSlider(Qt.Horizontal)
        self.thumbCarpal_slider.setMinimum(00)
        self.thumbCarpal_slider.setMaximum(90)
        self.thumbCarpal_slider.setValue(0)
        self.thumbCarpal_slider.setTickPosition(QSlider.TicksBelow)
        self.thumbCarpal_slider.setTickInterval(5)

        self.thumbCarpal_slider.valueChanged.connect(self.thumbCarpalValue)

        thumbMcp = QLabel('Thumb Mcp Joint')
        self.thumbMcp_slider = QSlider(Qt.Horizontal)
        self.thumbMcp_slider.setMinimum(00)
        self.thumbMcp_slider.setMaximum(90)
        self.thumbMcp_slider.setValue(0)
        self.thumbMcp_slider.setTickPosition(QSlider.TicksBelow)
        self.thumbMcp_slider.setTickInterval(5)

        self.thumbMcp_slider.valueChanged.connect(self.thumbMcpValue)

        grid = QGridLayout()
        grid.setSpacing(10)

        grid.addWidget(fingerMcp, 1, 0)
        grid.addWidget(self.fingerMcp_slider, 1, 1)
        grid.addWidget(thumbCarpal, 2, 0)
        grid.addWidget(self.thumbCarpal_slider, 2, 1)
        grid.addWidget(thumbMcp, 3, 0)
        grid.addWidget(self.thumbMcp_slider, 3, 1)

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