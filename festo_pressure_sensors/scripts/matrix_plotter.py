#!/usr/bin/env python
import sys, os, signal
from PyQt5 import uic
from PyQt5 import QtCore, QtGui, QtWidgets



os.system("python -m PyQt5.uic.pyuic pressure_sensors.ui -o pressure_sensor_gui.py -x")
from pressure_sensor_gui import Ui_MainWindow
from std_msgs.msg import Float32MultiArray, UInt16MultiArray
from festo_pressure_sensors.msg import SensorValuesArduino

import rospy


class PressureSensorGui(Ui_MainWindow, QtCore.QObject ):

    update_gui = QtCore.pyqtSignal(bool)
    msg = SensorValuesArduino()
    msg2 = Float32MultiArray()
    settings_msg = UInt16MultiArray()

    def __init__(self,mw):
        super(PressureSensorGui, self).__init__()
        self.setupUi(mw)
        self.mw = mw
        rospy.Subscriber("/adc", SensorValuesArduino, self.adc_cb)

        self.pub = rospy.Publisher("/chloe", Float32MultiArray, queue_size=1)
        self.settings_pub = rospy.Publisher("/settings", UInt16MultiArray, queue_size=1, latch=True)

        self.update_gui.connect(self.update_gui_cb)

        self.pushButton.clicked.connect(self.hello)

        mw.show()
        self.lbls = []
        self.x_dim = 0
        self.y_dim = 0

        self.settings_msg.data.append(5)  # Delay 1
        self.settings_msg.data.append(5)  # Delay 2
        self.settings_msg.data.append(155)  # Output voltage 1

        self.settings_pub.publish(self.settings_msg)

        self.hs_delay1.setValue(self.settings_msg.data[0])
        self.hs_delay2.setValue(self.settings_msg.data[1])
        self.sl_pwm_value.setValue(self.settings_msg.data[2])

        self.hs_delay1.valueChanged.connect(self.update_settings_cb)
        self.hs_delay2.valueChanged.connect(self.update_settings_cb)
        self.sl_pwm_value.valueChanged.connect(self.update_settings_cb)



    def update_settings_cb(self, event):

        self.settings_msg.data[0] = self.hs_delay1.value()
        self.settings_msg.data[1] = self.hs_delay2.value()
        self.settings_msg.data[2] = self.sl_pwm_value.value()
        self.settings_pub.publish(self.settings_msg)


    def setup_gui_matix(self):
        self.lbls = []
        for row in range(self.x_dim):

            for col in range(self.y_dim):
                lbl = QtWidgets.QLabel("")
                self.lbls.append(lbl)
                idx = self.y_dim * row + col
                self.gl_pmatrix.addWidget(self.lbls[idx], row, col, 1, 1)

    def hello(self):
        rospy.logerr("Hello")

    def pressure_color_mapping(self, pressure):
        """
        Returns a colour depending on the pressure
        """
        colour = QtGui.QColor()
        max_pressure = 1023.0
        default_colour = 0.0
        colour.a = (1 - ((max_pressure- pressure) / max_pressure))
        colour.r = default_colour
        colour.g = default_colour
        colour.b = default_colour

        return colour

    def adc_cb(self, msg):
        self.msg = msg
        self.update_gui.emit(True)

        self.msg2.data = self.msg.data
        self.pub.publish(self.msg2)

    def update_label(self, lbl, value):
        color = self.pressure_color_mapping(value)
        color_str = "rgba(%i,%i,%i,%i)" % (color.r * 255, color.g * 255, color.b * 255, color.a * 255)
        lbl.setStyleSheet( "background-color:" + color_str + ";border:1px solid " + color_str + ";border-radius:10px;")
        lbl.setNum(value)

    def filtered_data(self, idx):

        return self.msg.data[idx]

    def update_gui_cb(self):

        if not len(self.msg.data) is (self.msg.x_dim * self.msg.y_dim):
            rospy.logerr("Data size does not match dimension: dl %i size: %i" % (len(self.msg.data), self.x_dim * self.y_dim))
            return

        if self.y_dim is not self.msg.y_dim or self.x_dim is not self.msg.x_dim:
            self.x_dim = self.msg.x_dim
            self.y_dim = self.msg.y_dim
            self.setup_gui_matix()

        for row in range(self.x_dim):
            for col in range(self.y_dim):
                idx =self.y_dim*row + col
                self.update_label(self.lbls[idx], self.filtered_data(idx) )


if __name__ == '__main__':
    rospy.init_node("matrix_plotter_node")
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = PressureSensorGui(MainWindow)

    sys.exit(app.exec_())
