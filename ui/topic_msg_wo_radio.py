# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'topic_msg_wo_radio.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_topic_msg_wo_radio(object):
    def setupUi(self, topic_msg_wo_radio):
        topic_msg_wo_radio.setObjectName("topic_msg_wo_radio")
        topic_msg_wo_radio.resize(437, 712)
        topic_msg_wo_radio.setStyleSheet("background-color:white;")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(topic_msg_wo_radio)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.frame = QtWidgets.QFrame(topic_msg_wo_radio)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame_2 = QtWidgets.QFrame(self.frame)
        self.frame_2.setMaximumSize(QtCore.QSize(16777215, 30))
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.gridLayout = QtWidgets.QGridLayout(self.frame_2)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setSpacing(0)
        self.gridLayout.setObjectName("gridLayout")
        self.label = QtWidgets.QLabel(self.frame_2)
        self.label.setStyleSheet("background-color:yellow;")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.verticalLayout.addWidget(self.frame_2)
        self.frame_3 = QtWidgets.QFrame(self.frame)
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.frame_3)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setSpacing(0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.show_topic_msg_wo_radio = QtWidgets.QLabel(self.frame_3)
        self.show_topic_msg_wo_radio.setObjectName("show_topic_msg_wo_radio")
        self.verticalLayout_3.addWidget(self.show_topic_msg_wo_radio)
        self.verticalLayout.addWidget(self.frame_3)
        self.verticalLayout.setStretch(0, 1)
        self.verticalLayout.setStretch(1, 9)
        self.verticalLayout_2.addWidget(self.frame)

        self.retranslateUi(topic_msg_wo_radio)
        QtCore.QMetaObject.connectSlotsByName(topic_msg_wo_radio)

    def retranslateUi(self, topic_msg_wo_radio):
        _translate = QtCore.QCoreApplication.translate
        topic_msg_wo_radio.setWindowTitle(_translate("topic_msg_wo_radio", "Dialog"))
        self.label.setText(_translate("topic_msg_wo_radio", "description"))
        self.show_topic_msg_wo_radio.setText(_translate("topic_msg_wo_radio", "TextLabel"))
