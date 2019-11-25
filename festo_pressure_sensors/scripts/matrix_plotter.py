#!/usr/bin/env python
import sys, os, signal
from PyQt5 import uic
from PyQt5 import QtCore, QtGui, QtWidgets



os.system("python -m PyQt5.uic.pyuic pressure_sensors.ui -o pressure_sensor_gui.py -x")
from pressure_sensor_gui import Ui_MainWindow
from std_msgs.msg import Float32MultiArray, UInt16MultiArray
from festo_pressure_sensors.msg import SensorValuesArduino

import rospy
import numpy


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
        self.lbls_min = []
        self.lbls_max = []

        self.meas = []
        self.min = []
        self.max = []

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
        self.reset_minmax_matrix()


    def setup_gui_matrix(self):
        self.lbls = []
        for row in range(self.x_dim):
            for col in range(self.y_dim):
                lbl = QtWidgets.QLabel("")
                self.lbls.append(lbl)
                idx = self.y_dim * row + col
                self.gl_pmatrix.addWidget(self.lbls[idx], row, col, 1, 1)

    def setup_minmax_matrix(self):
        self.lbls_min = []
        self.lbls_max = []
        self.gl_minmax.setSpacing(1)

        for row in range(self.x_dim):
            for col in range(self.y_dim):

                lbl = QtWidgets.QLabel("")
                idx = self.y_dim * row + col

                self.lbls_min.append(lbl)
                self.min.append(1023)
                self.gl_minmax.addWidget(self.lbls_min[idx], row*2, col, 1, 1)

                lbl = QtWidgets.QLabel("")
                self.lbls_max.append(lbl)
                self.max.append(0)
                self.gl_minmax.addWidget(self.lbls_max[idx], row*2+1, col, 1.5, 1)

    def reset_minmax_matrix(self):
        for row in range(self.x_dim):
            for col in range(self.y_dim):
                idx = self.y_dim * row + col
                self.min[idx] = 1023
                self.max[idx] = 0
                self.update_label(self.lbls_min[idx], self.min[idx])
                self.update_label(self.lbls_max[idx], self.max[idx])


    def setup_gui_matrix_interp(self):
        self.lbls = []
        x_dim_interp = (self.x_dim + self.y_dim - 1)
        for row in range(x_dim_interp):
            for col in range(self.y_dim+self.y_dim-1):
                lbl = QtWidgets.QLabel("")
                self.meas.append(0)
                self.lbls.append(lbl)
                idx = x_dim_interp * row + col
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
        colour.a = max(0.0, min(1023.0, (1 - ((max_pressure- pressure) / max_pressure) )) )
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


    def filtered_data(self, row_i, col_i):
        filtered_val = 0
        x_dim_interp = (self.x_dim+self.y_dim-1)
        idx = x_dim_interp*row_i + col_i

        #Values between horizontal sensors:
        if (row_i%2 != 1) and (col_i%2 == 1):
            filtered_val = int((self.meas[idx-1] + self.meas[idx+1])/2)

        # Values between vertical sensors:
        if (row_i%2 == 1) and (col_i%2 != 1):
            filtered_val = int((self.meas[idx-x_dim_interp] + self.meas[idx+x_dim_interp])/2)

        #return self.msg.data[idx]
        return int(filtered_val)

    def update_gui_cb(self):

        if not len(self.msg.data) is (self.msg.x_dim * self.msg.y_dim):
            rospy.logerr("Data size does not match dimension: dl %i size: %i" % (len(self.msg.data), self.x_dim * self.y_dim))
            return

        if self.y_dim is not self.msg.y_dim or self.x_dim is not self.msg.x_dim:
            self.x_dim = self.msg.x_dim
            self.y_dim = self.msg.y_dim
            #self.setup_gui_matrix()
            self.setup_gui_matrix_interp()
            self.setup_minmax_matrix()

        # for row in range(self.x_dim+self.x_dim-1):
        #     for col in range(self.y_dim+self.y_dim-1):
        #         idx =(self.y_dim+self.y_dim-1)*row + col
        #         self.update_label(self.lbls[idx], idx)

        #Reset matrix
        x_dim_interp = (self.x_dim + self.y_dim)
        for row in range(x_dim_interp+1):
            for col in range(x_dim_interp):
                idx = self.y_dim * row + col
                self.meas[idx] = 0
                self.update_label(self.lbls[idx], self.meas[idx])

        #Measured values
        for row in range(self.x_dim):
            for col in range(self.y_dim):
                idx = self.y_dim * row + col
                idx_interp = (self.x_dim + self.y_dim - 1)*row* 2 + col*2
                self.meas[idx_interp] = self.msg.data[idx]
                self.update_label(self.lbls[idx_interp], self.meas[idx_interp])

                if self.msg.data[idx] > self.max[idx]:
                    self.max[idx] = self.msg.data[idx]
                    self.update_label(self.lbls_max[idx], self.max[idx])

                if self.msg.data[idx] < self.min[idx]:
                    self.min[idx] = self.msg.data[idx]
                    self.update_label(self.lbls_min[idx], self.min[idx])

        #Interpolated values
        x_dim_interp = (self.x_dim + self.y_dim - 1)
        for row in range(x_dim_interp):
            for col in range(x_dim_interp):
                idx =self.y_dim*row + col
                if(self.meas[idx] == 0):
                    self.meas[idx] = self.filtered_data(row,col)
                    #self.update_label(self.lbls[idx], self.meas[idx] )


if __name__ == '__main__':
    rospy.init_node("matrix_plotter_node")
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = PressureSensorGui(MainWindow)

    sys.exit(app.exec_())
