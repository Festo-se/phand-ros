
import signal, sys
import numpy as np
import math as m

import roslib; roslib.load_manifest('urdfdom_py')
import rospy
from urdf_parser_py.urdf import URDF

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

from festo_phand_msgs.srv import *
from festo_phand_msgs.msg import *


class HandPressureSlider(QSlider):
    valueUpdated = pyqtSignal(float)
    def __init__(self, joint_name, max_pressure):
        super(HandPressureSlider, self).__init__(Qt.Horizontal)
        self.joint_name = joint_name
        self.max_pressure = max_pressure

        self.setMinimumWidth(300)
        self.setMinimum(0)
        self.setMaximum(100)
        self.setTickPosition(0)
        self.setTickPosition(QSlider.TicksBelow)

        self.valueChanged[int].connect(self.map_value)

        self.setStyleSheet("QSlider::groove:horizontal { border: 1px solid #999999; height: 8px; background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #B1B1B1, stop:1 #c4c4c4); margin: 2px 0;}"
                           "QSlider::handle:horizontal { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #b4b4b4, stop:1 #8f8f8f); "
                           "border: 1px solid #5c5c5c;height:30px;width: 30px;margin: -5px 0; border-radius: 3px;}")

    @property
    def joint_value(self):
        return self.value() * self.max_pressure/100

    def map_value(self, value):
        self.valueUpdated.emit(self.joint_value)

class HandControllerGui(QWidget):

    handstate_update = pyqtSignal()

    def __init__(self):
        super(QWidget, self).__init__()
        self.last_time = 0
        self.send_threshold = 0.01
        self.hand_state = HandState()
        self.sliders = []
        self.pressure_lbls = []
        self.joint_pressures = []
        self.overall_slider = QSlider(Qt.Horizontal)
        self.overall_slider.setValue(100)

        self.overall_slider.valueChanged.connect(self.open_btn_click)

        self.gridLayout = QGridLayout(self)

        self.joints = ["Thumb deviation",
                       "Thumb lower",
                       "Pressure spring",
                       "Thumb upper",
                       "Index upper",
                       "Cylinder left",
                       "Index lower",
                       "Cylinder right",
                       "Middle&ring low",
                       "Index deviation",
                       "Middle&ring top",
                       "pinky",
                       ]
        self.low_joints = [ "Thumb lower",
                            "Thumb upper",
                            "Index upper",
                            "Index lower",
                            "Middle&ring low",
                            "Middle&ring top",
                            "pinky"
                       ]

        self.joint_limits = [5.0]*12

        self.setup_gui()

        rospy.Subscriber("festo/phand/state", HandState, self.hand_state_cb)

        self.set_pressure_topic = rospy.Publisher("festo/phand/set_pressures", SimpleFluidPressures, queue_size=1)

        self.handstate_update.connect(self.update_pressure_lables)

        self.show()


    def update_pressure_lables(self):
        for num, pressure in enumerate(self.hand_state.internal_sensors.actual_pressures.values):
            # self.pressure_lbls[num].setNum( round(pressure/1e5-1,2))
            self.pressure_lbls[num].setText("%.2f" % round(pressure/1e5-1, 2) )

    def hand_state_cb(self, msg):
        self.hand_state = msg
        self.handstate_update.emit()

    def setup_gui(self):

        for row, joint in enumerate(self.joints):
            lbl = QLabel(str(joint))
            lbl_value = QLabel()
            lbl_value.setNum(0.00)

            lbl_value.setMinimumWidth(50)
            lbl_value.setMaximumWidth(50)

            self.sliders.append(HandPressureSlider(str(joint) + "_open", self.joint_limits[row]))
            self.sliders[row].valueUpdated.connect(lbl_value.setNum)
            self.sliders[row].valueUpdated.connect(self.open_btn_click)

            self.gridLayout.addWidget(lbl, row, 0)
            self.gridLayout.addWidget(self.sliders[row], row, 1)
            self.gridLayout.addWidget(lbl_value, row, 2)

        for row, joint in enumerate(self.joints):
            lbl = QLabel(str(0.0))
            lbl.setMinimumWidth(50)
            lbl.setMaximumWidth(50)

            self.pressure_lbls.append(lbl)

            lbl_value = QLabel()
            lbl_value.setNum(0.00)
            lbl_value.setMinimumWidth(50)
            lbl_value.setMaximumWidth(50)

            self.sliders.append(HandPressureSlider(str(joint) + "_close", self.joint_limits[row]))
            self.sliders[row + len(self.joints)].valueUpdated.connect(lbl_value.setNum)
            self.sliders[row + len(self.joints)].valueUpdated.connect(self.close_btn_click)

            self.gridLayout.addWidget(self.sliders[row + len(self.joints)], row, 3)
            self.gridLayout.addWidget(lbl_value, row, 4)
            self.gridLayout.addWidget(lbl, row, 5)

        btn_open = QPushButton("Open")
        btn_open.clicked.connect(self.open_btn_click)
        btn_close = QPushButton("Close")
        btn_close.clicked.connect(self.close_btn_click)
        self.gridLayout.addWidget(btn_open, len(self.joints) + 1, 1)
        self.gridLayout.addWidget(btn_close, len(self.joints) + 1, 3)

        lbl = QLabel(str(0.0))
        self.pressure_lbls.append(lbl)
        self.gridLayout.addWidget(lbl, len(self.joints) + 1, 4)

        lbl = QLabel(str(0.0))
        self.pressure_lbls.append(lbl)
        self.gridLayout.addWidget(lbl, len(self.joints) + 1, 2)

        self.gridLayout.addWidget(self.overall_slider, len(self.joints) + 2, 0, 1,6)

        btn_copy_open = QPushButton("Copy pressure open values")
        btn_copy_open.clicked.connect(self.copy_values)
        self.gridLayout.addWidget(btn_copy_open, len(self.joints) + 3, 0, 1, 3)

        btn_copy_close = QPushButton("Copy pressure close values")
        btn_copy_close.clicked.connect(self.copy_values)
        self.gridLayout.addWidget(btn_copy_close, len(self.joints) + 3, 3, 1, 3)

    def copy_values(self):
        sender = self.sender()
        if "open" in sender.text():
            self.generate_publish_data("_open")
        else:
            self.generate_publish_data("_close")

        clipboard = QApplication.clipboard()
        clipboard.setText(str(self.joint_pressures))

    def generate_publish_data(self, suffix):
        self.joint_pressures = []
        for joint in self.sliders:
            if suffix in joint.joint_name:
                if joint.joint_name.replace(suffix,"") in self.low_joints:
                    self.joint_pressures.append((joint.joint_value+1)*1e5*self.overall_slider.value()/100)
                else:
                    self.joint_pressures.append((joint.joint_value + 1)*1e5 )

    def close_btn_click(self):

        if rospy.get_time() - self.last_time > self.send_threshold:
            self.last_time = rospy.get_time()
            self.generate_publish_data("_close")
            msg = SimpleFluidPressures()
            msg.values = self.joint_pressures
            self.set_pressure_topic.publish(msg)

    def open_btn_click(self):

        if rospy.get_time() - self.last_time > self.send_threshold:
            self.last_time = rospy.get_time()
            self.generate_publish_data("_open")
            msg = SimpleFluidPressures()
            msg.values = self.joint_pressures
            self.set_pressure_topic.publish(msg)


if __name__ == '__main__':
    rospy.init_node('joint_state_publisher')
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    ui = HandControllerGui()
    # Start the event loop.
    sys.exit(app.exec_())