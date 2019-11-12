# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'pressure_sensors.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(563, 490)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setObjectName("pushButton")
        self.gridLayout.addWidget(self.pushButton, 0, 2, 1, 1)
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout.addWidget(self.pushButton_2, 1, 2, 1, 1)
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setCheckable(True)
        self.pushButton_3.setObjectName("pushButton_3")
        self.gridLayout.addWidget(self.pushButton_3, 2, 2, 1, 1)
        self.hs_delay2 = QtWidgets.QSlider(self.centralwidget)
        self.hs_delay2.setOrientation(QtCore.Qt.Horizontal)
        self.hs_delay2.setObjectName("hs_delay2")
        self.gridLayout.addWidget(self.hs_delay2, 4, 2, 1, 1)
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setObjectName("groupBox")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.groupBox)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.gl_pmatrix = QtWidgets.QGridLayout()
        self.gl_pmatrix.setObjectName("gl_pmatrix")
        self.gridLayout_3.addLayout(self.gl_pmatrix, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.groupBox, 0, 0, 6, 1)
        self.hs_delay1 = QtWidgets.QSlider(self.centralwidget)
        self.hs_delay1.setOrientation(QtCore.Qt.Horizontal)
        self.hs_delay1.setObjectName("hs_delay1")
        self.gridLayout.addWidget(self.hs_delay1, 3, 2, 1, 1)
        self.sl_pwm_value = QtWidgets.QSlider(self.centralwidget)
        self.sl_pwm_value.setMaximum(255)
        self.sl_pwm_value.setOrientation(QtCore.Qt.Horizontal)
        self.sl_pwm_value.setObjectName("sl_pwm_value")
        self.gridLayout.addWidget(self.sl_pwm_value, 5, 2, 1, 1)
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setMinimumSize(QtCore.QSize(100, 0))
        self.label.setText("")
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 3, 3, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setMinimumSize(QtCore.QSize(100, 0))
        self.label_2.setText("")
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 4, 3, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setMinimumSize(QtCore.QSize(100, 0))
        self.label_3.setText("")
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 5, 3, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 3, 1, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 4, 1, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setObjectName("label_6")
        self.gridLayout.addWidget(self.label_6, 5, 1, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 563, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.hs_delay1.sliderMoved['int'].connect(self.label.setNum)
        self.hs_delay2.sliderMoved['int'].connect(self.label_2.setNum)
        self.sl_pwm_value.sliderMoved['int'].connect(self.label_3.setNum)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "No User"))
        self.pushButton_2.setText(_translate("MainWindow", "Human User"))
        self.pushButton_3.setText(_translate("MainWindow", "Robot User"))
        self.groupBox.setTitle(_translate("MainWindow", "Matrix"))
        self.label_4.setText(_translate("MainWindow", "Delay1"))
        self.label_5.setText(_translate("MainWindow", "Delay2"))
        self.label_6.setText(_translate("MainWindow", "pwm"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

