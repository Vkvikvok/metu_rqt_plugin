# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'sci_hub_controller.ui'
#
# Created by: PyQt5 UI code generator 5.15.5
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class SciHubControllerUi(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(400, 300)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(Form)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setSpacing(10)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(Form)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.drill_up_button = QtWidgets.QPushButton(Form)
        self.drill_up_button.setCheckable(False)
        self.drill_up_button.setObjectName("drill_up_button")
        self.verticalLayout.addWidget(self.drill_up_button)
        self.drill_stop_button = QtWidgets.QPushButton(Form)
        self.drill_stop_button.setObjectName("drill_stop_button")
        self.verticalLayout.addWidget(self.drill_stop_button)
        self.drill_down_button = QtWidgets.QPushButton(Form)
        self.drill_down_button.setObjectName("drill_down_button")
        self.verticalLayout.addWidget(self.drill_down_button)
        self.horizontalLayout_4.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_2 = QtWidgets.QLabel(Form)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_2.addWidget(self.label_2)
        self.sci_plat_up_button = QtWidgets.QPushButton(Form)
        self.sci_plat_up_button.setObjectName("sci_plat_up_button")
        self.verticalLayout_2.addWidget(self.sci_plat_up_button)
        self.sci_plat_stop_button = QtWidgets.QPushButton(Form)
        self.sci_plat_stop_button.setObjectName("sci_plat_stop_button")
        self.verticalLayout_2.addWidget(self.sci_plat_stop_button)
        self.sci_plat_down_button = QtWidgets.QPushButton(Form)
        self.sci_plat_down_button.setObjectName("sci_plat_down_button")
        self.verticalLayout_2.addWidget(self.sci_plat_down_button)
        self.horizontalLayout_4.addLayout(self.verticalLayout_2)
        self.verticalLayout_5.addLayout(self.horizontalLayout_4)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_6 = QtWidgets.QLabel(Form)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.verticalLayout_4.addWidget(self.label_6)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.drill_head_ccw_button = QtWidgets.QPushButton(Form)
        self.drill_head_ccw_button.setObjectName("drill_head_ccw_button")
        self.horizontalLayout_3.addWidget(self.drill_head_ccw_button)
        self.drill_head_stop_button = QtWidgets.QPushButton(Form)
        self.drill_head_stop_button.setObjectName("drill_head_stop_button")
        self.horizontalLayout_3.addWidget(self.drill_head_stop_button)
        self.drill_head_cw_button = QtWidgets.QPushButton(Form)
        self.drill_head_cw_button.setObjectName("drill_head_cw_button")
        self.horizontalLayout_3.addWidget(self.drill_head_cw_button)
        self.verticalLayout_4.addLayout(self.horizontalLayout_3)
        self.verticalLayout_5.addLayout(self.verticalLayout_4)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_5 = QtWidgets.QLabel(Form)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.verticalLayout_3.addWidget(self.label_5)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_3 = QtWidgets.QLabel(Form)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout.addWidget(self.label_3)
        self.rotate_60_deg_ccw_button = QtWidgets.QPushButton(Form)
        self.rotate_60_deg_ccw_button.setObjectName("rotate_60_deg_ccw_button")
        self.horizontalLayout.addWidget(self.rotate_60_deg_ccw_button)
        self.rotate_60_deg_cw_button = QtWidgets.QPushButton(Form)
        self.rotate_60_deg_cw_button.setObjectName("rotate_60_deg_cw_button")
        self.horizontalLayout.addWidget(self.rotate_60_deg_cw_button)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_4 = QtWidgets.QLabel(Form)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_2.addWidget(self.label_4)
        self.rotate_1_step_ccw_button = QtWidgets.QPushButton(Form)
        self.rotate_1_step_ccw_button.setObjectName("rotate_1_step_ccw_button")
        self.horizontalLayout_2.addWidget(self.rotate_1_step_ccw_button)
        self.rotate_1_step_cw_button = QtWidgets.QPushButton(Form)
        self.rotate_1_step_cw_button.setObjectName("rotate_1_step_cw_button")
        self.horizontalLayout_2.addWidget(self.rotate_1_step_cw_button)
        self.verticalLayout_3.addLayout(self.horizontalLayout_2)
        self.verticalLayout_5.addLayout(self.verticalLayout_3)
        self.horizontalLayout_5.addLayout(self.verticalLayout_5)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label.setText(_translate("Form", "Drill Control"))
        self.drill_up_button.setText(_translate("Form", "Up"))
        self.drill_stop_button.setText(_translate("Form", "Stop"))
        self.drill_down_button.setText(_translate("Form", "Down"))
        self.label_2.setText(_translate("Form", "Science Platform Control"))
        self.sci_plat_up_button.setText(_translate("Form", "Up"))
        self.sci_plat_stop_button.setText(_translate("Form", "Stop"))
        self.sci_plat_down_button.setText(_translate("Form", "Down"))
        self.label_6.setText(_translate("Form", "Drill Head Rotation"))
        self.drill_head_ccw_button.setText(_translate("Form", "C. Clockwise"))
        self.drill_head_stop_button.setText(_translate("Form", "Stop"))
        self.drill_head_cw_button.setText(_translate("Form", "Clockwise"))
        self.label_5.setText(_translate("Form", "Science Hub Rotation"))
        self.label_3.setText(_translate("Form", "60 degree"))
        self.rotate_60_deg_ccw_button.setText(_translate("Form", "C. Clockwise"))
        self.rotate_60_deg_cw_button.setText(_translate("Form", "Clockwise"))
        self.label_4.setText(_translate("Form", "1 step"))
        self.rotate_1_step_ccw_button.setText(_translate("Form", "C. Clockwise"))
        self.rotate_1_step_cw_button.setText(_translate("Form", "Clockwise"))
