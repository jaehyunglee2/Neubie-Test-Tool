# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'topic_msg.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_topic_msg(object):
    def setupUi(self, topic_msg):
        topic_msg.setObjectName("topic_msg")
        topic_msg.resize(744, 212)
        topic_msg.setStyleSheet("background-color:white;")
        self.gridLayout = QtWidgets.QGridLayout(topic_msg)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setSpacing(0)
        self.gridLayout.setObjectName("gridLayout")
        self.frame = QtWidgets.QFrame(topic_msg)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.frame)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.msg_indication = QtWidgets.QFrame(self.frame)
        self.msg_indication.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.msg_indication.setFrameShadow(QtWidgets.QFrame.Raised)
        self.msg_indication.setObjectName("msg_indication")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.msg_indication)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.title = QtWidgets.QFrame(self.msg_indication)
        self.title.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.title.setFrameShadow(QtWidgets.QFrame.Raised)
        self.title.setObjectName("title")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.title)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setSpacing(0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label = QtWidgets.QLabel(self.title)
        self.label.setStyleSheet("background-color:yellow;\n"
"")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.verticalLayout_3.addWidget(self.label)
        self.verticalLayout_2.addWidget(self.title)
        self.msg_indi = QtWidgets.QFrame(self.msg_indication)
        self.msg_indi.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.msg_indi.setFrameShadow(QtWidgets.QFrame.Raised)
        self.msg_indi.setObjectName("msg_indi")
        self.verticalLayout_2.addWidget(self.msg_indi)
        self.verticalLayout_2.setStretch(0, 1)
        self.verticalLayout_2.setStretch(1, 9)
        self.horizontalLayout.addWidget(self.msg_indication)
        self.msg_disc_frame = QtWidgets.QFrame(self.frame)
        self.msg_disc_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.msg_disc_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.msg_disc_frame.setObjectName("msg_disc_frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.msg_disc_frame)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame_4 = QtWidgets.QFrame(self.msg_disc_frame)
        self.frame_4.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.frame_4)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_4.setSpacing(0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_2 = QtWidgets.QLabel(self.frame_4)
        self.label_2.setStyleSheet("background-color:yellow;\n"
"")
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_4.addWidget(self.label_2)
        self.verticalLayout.addWidget(self.frame_4)
        self.frame_3 = QtWidgets.QFrame(self.msg_disc_frame)
        self.frame_3.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.frame_3)
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_5.setSpacing(0)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.desc = QtWidgets.QLabel(self.frame_3)
        self.desc.setText("")
        self.desc.setObjectName("desc")
        self.verticalLayout_5.addWidget(self.desc)
        self.verticalLayout.addWidget(self.frame_3)
        self.verticalLayout.setStretch(0, 1)
        self.verticalLayout.setStretch(1, 9)
        self.horizontalLayout.addWidget(self.msg_disc_frame)
        self.gridLayout.addWidget(self.frame, 0, 0, 1, 1)

        self.retranslateUi(topic_msg)
        QtCore.QMetaObject.connectSlotsByName(topic_msg)

    def retranslateUi(self, topic_msg):
        _translate = QtCore.QCoreApplication.translate
        topic_msg.setWindowTitle(_translate("topic_msg", "Dialog"))
        self.label.setText(_translate("topic_msg", "indication"))
        self.label_2.setText(_translate("topic_msg", "description"))