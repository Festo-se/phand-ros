#!/usr/bin/env python
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


class JointCalculations:

    def __init__(self):
        self.l1 = 27.63e-3
        self.l2 = 26.63e-3
        self.l3 = 158e-3
        self.l4 = 27.02e-3
        self.l5 = 26.95e-3
        self.theta3 = np.deg2rad(18.6)

    def calculate_wristBase_cylinderR(self, theta1, theta2):

        return -m.atan2(- m.cos(self.theta3)*(self.l4 - self.l1*m.cos(theta1)) - m.sin(self.theta3)*(self.l2*m.cos(theta2) - self.l5
               + self.l1*m.sin(theta1)*m.sin(theta2)),
                        m.sqrt(
                            m.pow(abs(self.l3 - self.l2*m.sin(theta2) + self.l1*m.cos(theta2)*m.sin(theta1)),2)
                         + m.pow(abs(self.l2*m.cos(theta2) - self.l5 + self.l1*m.sin(theta1)*m.sin(theta2)),2)
                            + m.pow(abs(self.l4 - self.l1*m.cos(theta1)),2)
                        ))
    # def calculate_horizontal_R_vertical_R:
    def calculate_leftcylinder_rod(self,theta1, theta2):

        return m.sqrt(
            m.pow(abs(self.l2*m.cos(theta2) - self.l5 + self.l1*m.sin(theta1)*m.sin(theta2)),2) +
            m.pow(abs(self.l4 - self.l1*m.cos(theta1)),2) +
            m.pow(abs(self.l3 - self.l2*m.sin(theta2) + self.l1*m.cos(theta2)*m.sin(theta1)),2)
                  )


class JointSlider(QSlider):

    valueUpdated = pyqtSignal(float)

    def __init__(self, joint_name, joint_data):
        super(JointSlider, self).__init__(Qt.Horizontal)
        self.joint_name = joint_name
        self.setMinimumWidth(300)
        self.setMinimum(-100)
        self.setMaximum(100)
        self.setTickPosition(0)
        self.setTickPosition(QSlider.TicksBelow)

        self.valueChanged[int].connect(self.map_value)
        self.joint_data = joint_data

        self.map_value(0.0)


    @property
    def joint_value(self):
        if self.value() < 0:
            return -1*((self.joint_data.limit.lower) / 100) * self.value()
        else:
            return ((self.joint_data.limit.upper) / 100) * self.value()


    def map_value(self, value):
        self.valueUpdated.emit(self.joint_value)




class HandJointPublisher(QWidget):

    def __init__(self):
        super(QWidget, self).__init__()

        self.gridLayout = QGridLayout(self)
        self.show()

        self.robot = URDF.from_parameter_server()
        print(self.robot)

        self.joint_state = JointState()
        self.jc = JointCalculations()
        self.display_joints = ['rotation_x', 'rotation_y']
        self.joint_state.name = [ 'rotation_x', 'rotation_y', 'pinky_joint_1', 'pinky_joint_6', 'ringjoint_1', 'ringjoint_6', 'middlejoint_1', 'middlejoint_6',
                                  'indexjoint_1', 'indexjoint_6', 'thumbjoint_1', 'thumbjoint_6', 'drive_rotation', 'cylinder_rod', 'leftcylinder_rod', 'rightcylinder_rod', 'index_deviation',
                                  'index_cylinder_base', 'index_cylinder_tip', 'wristBase_horizontal_L', 'horizontal_L_vertical_L',
                                  'wristBase_cylinderR', 'horizontal_R_vertical_R']
        self.joint_state.position = [0] * len(self.joint_state.name)
        self.sliders = []

        self.setup_gui()

        self.publish_joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)

        self.publish_joint_state_timer = QTimer()
        self.publish_joint_state_timer.timeout.connect(self.update_joint_state)
        self.publish_joint_state_timer.start(100)



    def update_joint_state(self):
        self.joint_state.header.stamp = rospy.Time.now()

        for row, slider in enumerate(self.sliders):
            self.joint_state.position[row] = slider.joint_value

        # other joint mapping
        idx_theta1 = self.joint_state.name.index("rotation_x")
        idx_theta2 = self.joint_state.name.index("rotation_y")

        theta1 = self.joint_state.position[idx_theta1]
        theta2 = self.joint_state.position[idx_theta2]
        idx_wbr = self.joint_state.name.index("wristBase_cylinderR")
        idx_lcr = self.joint_state.name.index("leftcylinder_rod")

        self.joint_state.position[idx_wbr] = self.jc.calculate_wristBase_cylinderR(theta1,theta2)


        self.joint_state.position[idx_lcr] = self.jc.calculate_leftcylinder_rod(theta1,theta2) - 0.1580



        print(self.joint_state.position[idx_wbr])

        self.publish_joint_state.publish(self.joint_state)



    def find_joint_data(self, name):

        for joint_data in self.robot.joints:
            if joint_data.name == name:
                return joint_data

    def setup_gui(self):


        for row, joint in enumerate(self.joint_state.name):


            lbl = QLabel(joint)
            lbl_value = QLabel()
            lbl_value.setNum(0.00)
            lbl_value.setMinimumWidth(50)
            lbl_value.setMaximumWidth(50)
            joint_data = self.find_joint_data(joint)
            self.sliders.append(JointSlider(joint, joint_data))
            self.sliders[row].valueUpdated.connect(lbl_value.setNum)

            if joint in self.display_joints:
                self.gridLayout.addWidget(lbl,row,0)
                self.gridLayout.addWidget(self.sliders[row], row, 1)
                self.gridLayout.addWidget(lbl_value, row, 2)





if __name__ == '__main__':
    rospy.init_node('festo_phand_joint_state_publisher')
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    ui = HandJointPublisher()
    # Start the event loop.
    sys.exit(app.exec_())