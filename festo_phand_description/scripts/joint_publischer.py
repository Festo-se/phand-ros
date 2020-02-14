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

        self.l9     = 76.41e-3
        self.l10    = 18.81e-3
        self.l11_0  = self.l9-self.l10

        self.theta3 = np.deg2rad(18.6)

    # Matlab theta 4
    def calculate_wristBase_cylinderR(self, theta1, theta2):

        return -m.atan2(- m.cos(self.theta3)*(self.l4 - self.l1*m.cos(theta1)) - m.sin(self.theta3)*(self.l2*m.cos(theta2) - self.l5
               + self.l1*m.sin(theta1)*m.sin(theta2)),
                        m.sqrt(
                            m.pow(abs(self.l3 - self.l2*m.sin(theta2) + self.l1*m.cos(theta2)*m.sin(theta1)),2)
                         + m.pow(abs(self.l2*m.cos(theta2) - self.l5 + self.l1*m.sin(theta1)*m.sin(theta2)),2)
                            + m.pow(abs(self.l4 - self.l1*m.cos(theta1)),2)
                        ))

    # Matlab theta5
    def calculate_horizontal_R_vertical_R(self,theta1, theta2):

        return m.atan2(
            m.cos(self.theta3)*(self.l2*m.cos(theta2) - self.l5 +
            self.l1*m.sin(theta1)*m.sin(theta2)) -
            m.sin(self.theta3)*(self.l4 - self.l1*m.cos(theta1)),
            self.calculate_rigthcylinder_rod(theta1,theta2) # L6
        )

    # Matlab theta 6
    def calculate_wristBase_cylinderL(self, theta1, theta2):

        return -m.atan2(
            -m.cos(-self.theta3)*(self.l4 - self.l1*m.cos(theta1))
            -m.sin(-self.theta3)*(self.l2*m.cos(theta2)
            -self.l5 + self.l1*m.sin(theta1)*m.sin(theta2)),
            self.calculate_leftcylinder_rod(theta1, theta2))


    # Matlab theta7
    def calculate_horizontal_L_vertical_L(self,theta1, theta2):

        return m.atan2(
            m.cos(-self.theta3)*(self.l2*m.cos(theta2)
            - self.l5 + self.l1*m.sin(theta1)*m.sin(theta2))
            - m.sin(-self.theta3)*(self.l4 - self.l1*m.cos(theta1)),
            self.calculate_leftcylinder_rod(theta1, theta2))


    def calculate_l0(self):
        return self.calculate_leftcylinder_rod(0,0)

    # Matlab L6
    def calculate_rigthcylinder_rod(self,theta1, theta2):

        return m.sqrt(
            m.pow(abs(self.l4 - self.l1*m.cos(theta1)),2) +
            m.pow(abs(self.l3 + self.l2*m.sin(theta2) + self.l1*m.cos(theta2)*m.sin(theta1)),2) +
            m.pow(abs(self.l5 - self.l2*m.cos(theta2) + self.l1*m.sin(theta1)*m.sin(theta2)),2)
        )

    # Matlab L7
    def calculate_leftcylinder_rod(self,theta1, theta2):

        return m.sqrt(
            m.pow(abs(self.l2*m.cos(theta2) - self.l5 + self.l1*m.sin(theta1)*m.sin(theta2)),2) +
            m.pow(abs(self.l4 - self.l1*m.cos(theta1)),2) +
            m.pow(abs(self.l3 - self.l2*m.sin(theta2) + self.l1*m.cos(theta2)*m.sin(theta1)),2)
                  )

    def calculate_index_angles(self, cylinder_rod ):



        self.l11 = self.l11_0+cylinder_rod

        ph1 =  2*m.atan(((self.l9*m.pow((self.l9 + self.l10 - self.l11),2)*m.sqrt(((self.l9 - self.l10 + self.l11)*(self.l10 - self.l9 + self.l11))/( m.pow((self.l9 + self.l10 - self.l11),3)*(self.l9 + self.l10 + self.l11))))/(self.l9 - self.l10 + self.l11) - (self.l10*m.pow((self.l9 + self.l10 - self.l11),2)*m.sqrt(((self.l9 - self.l10 + self.l11)*(self.l10 - self.l9 + self.l11))/(m.pow((self.l9 + self.l10 - self.l11),3)*(self.l9 + self.l10 + self.l11))))/(self.l9 - self.l10 + self.l11) + (self.l11*m.pow((self.l9 + self.l10 - self.l11),2)*m.sqrt(((self.l9 - self.l10 + self.l11)*(self.l10 - self.l9 + self.l11))/(m.pow((self.l9 + self.l10 - self.l11),3)*(self.l9 + self.l10 + self.l11))))/(self.l9 - self.l10 + self.l11))/(self.l9 + self.l10 - self.l11))

        ph2 = 2*m.atan(m.pow((self.l9 + self.l10 - self.l11),2)*m.sqrt((((self.l9 - self.l10 + self.l11)*(self.l10 - self.l9 + self.l11))/(m.pow( (self.l9 + self.l10 - self.l11), 3)*(self.l9 + self.l10 + self.l11))))/(self.l9 - self.l10 + self.l11))

        print([cylinder_rod, ph1, ph1])

        return [ph1, ph2]



class JointSlider(QSlider):

    valueUpdated = pyqtSignal(float)

    def __init__(self, joint_name, joint_data, joint_list):
        super(JointSlider, self).__init__(Qt.Horizontal)
        self.joint_list = joint_list
        self.joint_name = joint_name
        self.setMinimumWidth(300)
        self.setMinimum(0)
        self.setMaximum(100)
        self.setTickPosition(0)
        self.setTickPosition(QSlider.TicksBelow)

        self.valueChanged[int].connect(self.map_value)
        self.joint_data = joint_data

        print(joint_data)
        self.set_zero_value()
        self.map_value(self.value())

    @property
    def joint_value(self):

        if self.joint_data.mimic is None:
            return ((self.joint_data.limit.upper - self.joint_data.limit.lower ) / 100) * self.value() + self.joint_data.limit.lower
        else:

            for joint in self.joint_list:
                if joint.joint_name in self.joint_data.mimic.joint:
                    mul = 1
                    offset = 0

                    if type(self.joint_data.mimic.multiplier) is float:
                        mul = self.joint_data.mimic.multiplier

                    if type(self.joint_data.mimic.offset) is float:
                        offset = self.joint_data.mimic.offset

                    return joint.joint_value*mul + offset
            rospy.logwarn("Mimic joint not found from: %s to: %s "%(self.joint_name, self.joint_data.mimic.joint))
            return 0.0

    def set_zero_value(self):
        print([self.joint_data.name, self.joint_data.limit.lower, self.joint_data.limit.upper])
        zero_value = 0
        if self.joint_data.limit.lower > 0:
            zero_value = self.joint_data.limit.lower
        else:
            zero_value = -self.joint_data.limit.lower / ((self.joint_data.limit.upper - self.joint_data.limit.lower ) / 100)

        self.setValue(zero_value)

    def map_value(self, value):
        self.valueUpdated.emit(self.joint_value)


class HandJointPublisher(QWidget):

    def __init__(self):
        super(QWidget, self).__init__()

        self.hand_prefix = rospy.get_param("~hand_name", "")
        self.display_joints = rospy.get_param("~visible_joints", "")
        self.joint_wild_card = rospy.get_param("~show_all_joints_with", "phand_")
        self.not_display_joints = [
            self.hand_prefix +'rightcylinder_rod',
            self.hand_prefix +'wristBase_cylinderR',
            self.hand_prefix +'horizontal_R_vertical_R',
            self.hand_prefix +'leftcylinder_rod',
            self.hand_prefix +'wristBase_horizontal_L',
            self.hand_prefix +'horizontal_L_vertical_L',
            self.hand_prefix +'index_deviation',
            self.hand_prefix +'index_cylinder_base',
        ]

        self.gridLayout = QGridLayout(self)
        self.setWindowTitle(self.hand_prefix + "joint publisher")
        self.show()

        self.robot = URDF.from_parameter_server()

        self.joint_state = JointState()
        self.jc = JointCalculations()
        self.display_joints.extend([ self.hand_prefix +'rotation_x', self.hand_prefix +'rotation_y', self.hand_prefix +'cylinder_rod'])
        self.joint_state.name = []

        self.generate_joint_list()
        self.joint_state.position = [0] * len(self.joint_state.name)
        self.sliders = []

        self.setup_gui()

        self.publish_joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)

        self.publish_joint_state_timer = QTimer()
        self.publish_joint_state_timer.timeout.connect(self.update_joint_state)
        self.publish_joint_state_timer.start(100)

    def generate_joint_list(self):

        for joint in self.robot.joints:
            if joint.type in ["revolute","prismatic"]:
                self.joint_state.name.append(joint.name)

    def update_joint_state(self):
        self.joint_state.header.stamp = rospy.Time.now()

        for row, slider in enumerate(self.sliders):
            self.joint_state.position[row] = slider.joint_value

        # other joint mapping
        idx_theta1 = self.joint_state.name.index(self.hand_prefix +"rotation_x")
        idx_theta2 = self.joint_state.name.index(self.hand_prefix +"rotation_y")
        theta1 = self.joint_state.position[idx_theta1]
        theta2 = self.joint_state.position[idx_theta2]

        # Right cylinder
        idx_rcr = self.joint_state.name.index(self.hand_prefix +"rightcylinder_rod")
        idx_wbr = self.joint_state.name.index(self.hand_prefix +"wristBase_cylinderR")
        idx_hrvr = self.joint_state.name.index(self.hand_prefix +"horizontal_R_vertical_R")
        self.joint_state.position[idx_wbr] = -self.jc.calculate_wristBase_cylinderR(theta1,theta2)
        self.joint_state.position[idx_hrvr] = -self.jc.calculate_horizontal_R_vertical_R(theta1,theta2)
        self.joint_state.position[idx_rcr] = self.jc.calculate_l0() - self.jc.calculate_rigthcylinder_rod(theta1, theta2)

        # Left cylinder
        idx_lcr = self.joint_state.name.index(self.hand_prefix +"leftcylinder_rod")
        idx_wbl = self.joint_state.name.index(self.hand_prefix +"wristBase_horizontal_L")
        idx_hrvl = self.joint_state.name.index(self.hand_prefix +"horizontal_L_vertical_L")
        self.joint_state.position[idx_wbl] = self.jc.calculate_wristBase_cylinderL(theta1,theta2)
        self.joint_state.position[idx_hrvl] = self.jc.calculate_horizontal_L_vertical_L(theta1,theta2)
        self.joint_state.position[idx_lcr] = self.jc.calculate_leftcylinder_rod(theta1,theta2) - self.jc.calculate_l0()

        # Calculate the index joints
        idx_irod = self.joint_state.name.index(self.hand_prefix +"cylinder_rod")
        idx_idev = self.joint_state.name.index(self.hand_prefix +"index_deviation")
        idx_ibase = self.joint_state.name.index(self.hand_prefix +"index_cylinder_base")

        [phi1, phi2] = self.jc.calculate_index_angles(self.joint_state.position[idx_irod])
        self.joint_state.position[idx_idev] = phi1
        self.joint_state.position[idx_ibase] =  phi2

        print([self.joint_state.position[idx_rcr], self.joint_state.position[idx_irod]])
        print([self.joint_state.position[idx_rcr], self.joint_state.position[idx_irod]])

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


            self.sliders.append(JointSlider(joint, joint_data, self.sliders))
            self.sliders[row].valueUpdated.connect(lbl_value.setNum)

            if joint not in self.not_display_joints and ( joint in self.display_joints or ( self.joint_wild_card in joint and joint_data.mimic is None) ):
                self.gridLayout.addWidget(lbl,row,0)
                self.gridLayout.addWidget(self.sliders[row], row, 1)
                self.gridLayout.addWidget(lbl_value, row, 2)















if __name__ == '__main__':
    rospy.init_node('joint_state_publisher')
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    ui = HandJointPublisher()
    # Start the event loop.
    sys.exit(app.exec_())