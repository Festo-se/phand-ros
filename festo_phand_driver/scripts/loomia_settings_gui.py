#!/usr/bin/env python
import signal
import rospy
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from festo_phand_msgs.srv import *


class SettingsSlider(QSlider):
    valueUpdated = pyqtSignal(float)

    def __init__(self, name, maxv, default, minv):
        super(SettingsSlider, self).__init__(Qt.Horizontal)
        self.minv = minv
        self.maxv = maxv
        self.name = name
        self.setMinimumWidth(300)
        self.setMinimum(0)
        self.setMaximum(100)
        self.setTickPosition(0)
        self.setTickPosition(QSlider.TicksBelow)

        self.valueChanged[int].connect(self.map_value)

        self.set_slider_value(default)

    def set_slider_value(self, value):
        val = (value - self.minv)*100/(self.maxv-self.minv)
        self.setValue(val)

    @property
    def slider_value(self):
        return max(self.minv + self.value() * (self.maxv-self.minv) / 100, self.minv)

    def map_value(self):
        self.valueUpdated.emit(self.slider_value)


class LoomiaSettings(QWidget):

    def __init__(self):
        super(QWidget, self).__init__()

        self.gridLayout = QGridLayout(self)

        labels = [["Reference Voltage", 3.2, 2.0, 1.2],
                  ["Delay before measurement", 200, 75, 20],
                  ["Column 1",  50000, 40000, 0],
                  ["Column 2",  50000, 40000, 0],
                  ["Column 3",  50000, 40000, 0],
                  ["Column 4",  50000, 40000, 0],
                  ["Column 5",  50000, 40000, 0],
                  ["Column 6",  50000, 40000, 0],
                  ["Column 7",  50000, 40000, 0],
                  ["Column 8",  50000, 40000, 0],
                  ["Column 9",  50000, 40000, 0],
                  ["Column 10", 50000, 40000, 0],
                  ["Column 11", 50000, 40000, 0]
                  ]
        self.sliders = []
        for row, l in enumerate(labels):

            lbl = QLabel(l[0])
            lbl_value = QLabel("")
            lbl_value.setMinimumWidth(100)

            self.sliders.append(SettingsSlider(l[0], l[1], l[2], l[3]))

            self.sliders[row].valueUpdated.connect(self.value_updated)
            self.sliders[row].valueUpdated.connect(lbl_value.setNum)

            self.gridLayout.addWidget(lbl, row, 0)
            self.gridLayout.addWidget(self.sliders[row], row, 1)
            self.gridLayout.addWidget(lbl_value, row, 2)

        rospy.wait_for_service("festo/phand/loomia/set_configuration")
        self.set_loomia_settings = rospy.ServiceProxy("festo/phand/loomia/set_configuration", LoomiaSensorConfig)

        for sl in self.sliders:
            sl.map_value()

        self.show()

    def value_updated(self):

        msg = LoomiaSensorConfigRequest()
        msg.reference_voltage = self.sliders[0].slider_value
        msg.d_column_switch = self.sliders[1].slider_value
        for i in range(11):
            msg.series_resistance.append(self.sliders[2+i].slider_value)

        # print(msg)
        self.set_loomia_settings(msg)


if __name__ == '__main__':

    rospy.init_node("loomia_settings")
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    ui = LoomiaSettings()
    # Start the event loop.
    sys.exit(app.exec_())
