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


class handJointController(QWidget, QThread):
    def __init__(self):
        super(self.__class__, self).__init__()

        self.initROS()

        self.initUI()

        self.inc = 0
        self.index = 0.0
        self.middle = 0.0

        self.timer = QTimer()
        self.timer.timeout.connect(self.publishJointState)
        self.timer.start(100) #10 Hz

    def publishJointState(self):
        if not rospy.is_shutdown():
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = ['indexMCP', 'middleMCP']
            joint_state.position = [self.index * degToRad, self.middle * degToRad]
            joint_state.velocity = []
            joint_state.effort = []

            self.pub.publish(joint_state)
        else:
            print("ROS is shutdown")

    def closeEvent(self, event):
        self.timer.stop()

    def indexValue(self):
        self.index = float(self.indexMcp_slider.value())

    def middleValue(self):
        self.middle = float(self.middleMcp_slider.value())

    def initROS(self):
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        rospy.init_node('joint_state_publisher', anonymous=True)

    def initUI(self):
        indexMcp = QLabel('Index MCP Joint')
        self.indexMcp_slider = QSlider(Qt.Horizontal)
        self.indexMcp_slider.setMinimum(00)
        self.indexMcp_slider.setMaximum(90)
        self.indexMcp_slider.setValue(0)
        self.indexMcp_slider.setTickPosition(QSlider.TicksBelow)
        self.indexMcp_slider.setTickInterval(5)

        self.indexMcp_slider.valueChanged.connect(self.indexValue)

        middleMcp = QLabel('Middle MCP Joint')
        self.middleMcp_slider = QSlider(Qt.Horizontal)
        self.middleMcp_slider.setMinimum(00)
        self.middleMcp_slider.setMaximum(90)
        self.middleMcp_slider.setValue(0)
        self.middleMcp_slider.setTickPosition(QSlider.TicksBelow)
        self.middleMcp_slider.setTickInterval(5)

        self.middleMcp_slider.valueChanged.connect(self.middleValue)

        grid = QGridLayout()
        grid.setSpacing(10)

        grid.addWidget(indexMcp, 1, 0)
        grid.addWidget(self.indexMcp_slider, 1, 1)
        grid.addWidget(middleMcp, 2, 0)
        grid.addWidget(self.middleMcp_slider, 2, 1)

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