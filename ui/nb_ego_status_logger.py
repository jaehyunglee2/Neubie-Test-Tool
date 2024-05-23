# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'nb_ego_status_logger.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Topic_dialog(object):
    def setupUi(self, Topic_dialog):
        Topic_dialog.setObjectName("Topic_dialog")
        Topic_dialog.resize(325, 157)
        Topic_dialog.setStyleSheet("background-color:white;")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(Topic_dialog)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.msg_frame = QtWidgets.QFrame(Topic_dialog)
        self.msg_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.msg_frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.msg_frame.setObjectName("msg_frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.msg_frame)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.msg_frame)
        self.label.setMinimumSize(QtCore.QSize(0, 30))
        self.label.setMaximumSize(QtCore.QSize(16777215, 30))
        self.label.setStyleSheet("background-color:black;\n"
"color:white;")
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.rmt_ctl_flags_chkBox = QtWidgets.QCheckBox(self.msg_frame)
        self.rmt_ctl_flags_chkBox.setObjectName("rmt_ctl_flags_chkBox")
        self.verticalLayout.addWidget(self.rmt_ctl_flags_chkBox)
        self.distance_data_chkBox = QtWidgets.QCheckBox(self.msg_frame)
        self.distance_data_chkBox.setObjectName("distance_data_chkBox")
        self.verticalLayout.addWidget(self.distance_data_chkBox)
        self.verticalLayout_2.addWidget(self.msg_frame)

        self.retranslateUi(Topic_dialog)
        QtCore.QMetaObject.connectSlotsByName(Topic_dialog)

    def retranslateUi(self, Topic_dialog):
        _translate = QtCore.QCoreApplication.translate
        Topic_dialog.setWindowTitle(_translate("Topic_dialog", "Dialog"))
        self.label.setText(_translate("Topic_dialog", "<html><head/><body><p align=\"center\">메시지 선택</p></body></html>"))
        self.rmt_ctl_flags_chkBox.setText(_translate("Topic_dialog", "rmt_control_attention_flags (관제개입)"))
        self.distance_data_chkBox.setText(_translate("Topic_dialog", "distance_data"))
