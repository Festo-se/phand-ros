#!/usr/bin/env python
import sys
import os
import signal
import rospy
import numpy
import scipy.interpolate as sp_interp
from PyQt5 import uic
from PyQt5 import QtCore, QtGui, QtWidgets
from numpy import interp

import numpy as np
import scipy.ndimage as ndimage

os.system("python -m PyQt5.uic.pyuic pressure_sensors.ui -o pressure_sensor_gui.py -x")
from pressure_sensor_gui import Ui_MainWindow
from std_msgs.msg import Float32MultiArray, UInt16MultiArray
from festo_pressure_sensors.msg import SensorValuesArduino

class PressureSensorGui(Ui_MainWindow, QtCore.QObject ):

    update_gui = QtCore.pyqtSignal(bool)
    msg = SensorValuesArduino()
    settings_msg = UInt16MultiArray()

    def __init__(self, mw):
        super(PressureSensorGui, self).__init__()
        self.setupUi(mw)
        self.mw = mw

        rospy.Subscriber("/adc", SensorValuesArduino, self.adc_cb)
        self.settings_pub = rospy.Publisher("/settings", UInt16MultiArray, queue_size=1, latch=True)

        self.update_gui.connect(self.update_gui_cb)

        self.pushButton.clicked.connect(self.hello)
        self.pushButton_5.clicked.connect(self.reset_minmax_matrix)

        self.rb_interp.clicked.connect(self.setup_gui_matrix)
        self.rb_sensval.clicked.connect(self.setup_gui_matrix)
        self.cb_showVals.stateChanged.connect(self.setup_gui_matrix)

        mw.show()
        self.lbls = []
        self.lbls_min = []
        self.lbls_max = []

        self.meas_raw = []  # original x*y matrix size
        self.meas_interp = []
        self.meas_remap = []
        self.min = []
        self.max = []

        self.x_dim = 0
        self.y_dim = 0
        self.zoom = 2
        self.x_dim_interp = (self.x_dim * self.zoom)
        self.y_dim_interp = (self.y_dim * self.zoom)

        self.settings_msg.data.append(5)  # Delay 1
        self.settings_msg.data.append(5)  # Delay 2
        self.settings_msg.data.append(155)  # Output voltage 1

        self.settings_pub.publish(self.settings_msg)

        self.hs_delay1.setValue(self.settings_msg.data[0])
        self.lbl_delay1.setNum(self.settings_msg.data[0])
        self.hs_delay2.setValue(self.settings_msg.data[1])
        self.lbl_delay2.setNum(self.settings_msg.data[1])
        self.sl_pwm_value.setValue(self.settings_msg.data[2])
        self.lbl_pwm.setNum(self.settings_msg.data[2])

        self.hs_delay1.valueChanged.connect(self.update_settings_cb)
        self.hs_delay2.valueChanged.connect(self.update_settings_cb)
        self.sl_pwm_value.valueChanged.connect(self.update_settings_cb)
        self.sl_zoom.valueChanged.connect(self.setup_gui_matrix)

    def update_settings_cb(self, event):
        self.settings_msg.data[0] = self.hs_delay1.value()
        self.settings_msg.data[1] = self.hs_delay2.value()
        self.settings_msg.data[2] = self.sl_pwm_value.value()
        self.settings_pub.publish(self.settings_msg)
        self.reset_minmax_matrix()

    def setup_array_raw(self):
        for row in range(self.x_dim):
            for col in range(self.y_dim):
                self.meas_raw.append(0)
                self.meas_remap.append(0)
        rospy.logerr("raw values array setup")

    def setup_array_interp(self):
        self.zoom = self.sl_zoom.value()
        self.lbl_zoom.setText("{0:.1g}".format(self.zoom))
        self.x_dim_interp = (self.x_dim * self.zoom)
        self.y_dim_interp = (self.y_dim * self.zoom)

        self.meas_interp = []
        for row in range(self.x_dim_interp):
            for col in range(self.y_dim_interp):
                self.meas_interp.append(0)
        rospy.logerr("interpolated values array setup, array size: %d", len(self.meas_interp))

    def setup_gui_matrix(self):
        self.delete_gui_matrix()
        self.lbls = []

        if self.rb_interp.isChecked():
            self.setup_array_interp()
            x_range = self.x_dim_interp
            y_range = self.y_dim_interp
            rospy.logerr("GUI interpolation matrix setup")
        else:
            x_range = self.x_dim
            y_range = self.y_dim
            rospy.logerr("GUI std matrix setup")

        for row in range(x_range):
            for col in range(y_range):
                lbl = QtWidgets.QLabel("")
                self.lbls.append(lbl)
                idx = y_range * row + col
                self.gl_pmatrix.addWidget(self.lbls[idx], row, col, 1, 1)

    def delete_gui_matrix(self):
        for row in range(self.gl_pmatrix.rowCount()):
            for col in range (self.gl_pmatrix.columnCount()):
                item = self.gl_pmatrix.itemAtPosition(row, col)
                if item is not None:
                    widget = item.widget()
                    if isinstance(widget, QtWidgets.QLabel):
                        self.gl_pmatrix.removeWidget(widget)
                        widget.deleteLater()
                        widget = None

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

        self.reset_minmax_matrix()

    def reset_minmax_matrix(self):
        for row in range(self.x_dim):
            for col in range(self.y_dim):
                idx = self.y_dim * row + col
                self.min[idx] = 1023
                self.max[idx] = 0
                self.update_label(self.lbls_min[idx], self.min[idx], showVal=True)
                self.update_label(self.lbls_max[idx], self.max[idx], showVal=True)

    def hello(self):
        rospy.logerr("Hello")

    def pressure_color_mapping(self, pressure):
        """
        Returns a colour depending on the pressure
        """
        colour = QtGui.QColor()
        max_pressure = 1023.0

        default_colour = 0.0
        press_colour = max(0.05, min(1, (1 - ((max_pressure - pressure) / max_pressure))))
        colour.a = press_colour
        colour.r = press_colour-0.05
        colour.g = default_colour
        colour.b = 1-press_colour

        return colour

    def adc_cb(self, msg):
        self.msg = msg
        self.update_gui.emit(True)

    def update_label(self, lbl, value, showVal=1):
        color = self.pressure_color_mapping(value)
        color_str = "rgba(%i,%i,%i,%i)" % (color.r * 255, color.g * 255, color.b * 255, color.a * 255)
        lbl.setStyleSheet("background-color:" + color_str + ";border:1px solid " + color_str + ";border-radius:10px;")
        if showVal:
            lbl.setNum(value)

    # Remaps measured values from [min, max] to [0, 1023]
    def remapped_val(self, idx):
        if self.min[idx] == 1023 or self.max[idx] == 0:
            return 0
        else:
            val = int(interp(self.msg.data[idx], [self.min[idx], self.max[idx]], [0, 1023]))
            val = np.maximum(0, val)
            return val

    def remap_array(self):
        for idx in range(len(self.meas_raw)):
            self.meas_remap[idx] = self.remapped_val(idx)

    def update_gui_cb(self):

        if not len(self.msg.data) is (self.msg.x_dim * self.msg.y_dim):
            rospy.logerr("Data size does not match dimension: dl %i size: %i" % (len(self.msg.data), self.x_dim * self.y_dim))
            return

        if self.y_dim is not self.msg.y_dim or self.x_dim is not self.msg.x_dim:
            self.x_dim = self.msg.x_dim
            self.y_dim = self.msg.y_dim
            self.x_dim_interp = (self.x_dim * self.zoom)
            self.y_dim_interp = (self.y_dim * self.zoom)

            self.setup_array_raw()
            self.setup_array_interp()
            self.delete_gui_matrix()
            self.setup_gui_matrix()
            self.setup_minmax_matrix()

        # Measured values
        for row in range(self.x_dim):
            for col in range(self.y_dim):
                idx = self.y_dim * row + col
                self.meas_raw[idx] = self.msg.data[idx]
                if self.rb_remap.isChecked():
                    self.update_label(self.lbls[idx], self.remapped_val(idx), self.cb_showVals.isChecked())
                elif self.rb_sensval.isChecked():
                    self.update_label(self.lbls[idx], self.meas_raw[idx], self.cb_showVals.isChecked())

                # Adjust Max value
                if self.msg.data[idx] > self.max[idx]:
                    self.max[idx] = self.msg.data[idx]
                    self.update_label(self.lbls_max[idx], self.max[idx], showVal=True)

                # Adjust Min value
                if self.msg.data[idx] < self.min[idx]:
                    self.min[idx] = self.msg.data[idx]
                    self.update_label(self.lbls_min[idx], self.min[idx], showVal=True)

        # SciPy Zoom interpolation
        if self.rb_interp.isChecked():

            if self.rb_remap.isChecked():
                self.remap_array()
                data = np.reshape(self.meas_remap, (self.x_dim, self.y_dim))
            else:
                data = np.reshape(self.meas_raw, (self.x_dim, self.y_dim))

            self.meas_interp = ndimage.zoom(data, self.zoom)

            for row in range(self.x_dim_interp):
                for col in range(self.y_dim_interp):
                    idx = self.y_dim_interp * row + col
                    self.update_label(self.lbls[idx], np.maximum(self.meas_interp[row][col], 0), self.cb_showVals.isChecked() )


if __name__ == '__main__':
    rospy.init_node("matrix_plotter_node")
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = PressureSensorGui(MainWindow)

    sys.exit(app.exec_())
