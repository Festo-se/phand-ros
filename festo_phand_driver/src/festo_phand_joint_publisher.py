#!/usr/bin/env python
import signal, sys
import numpy as np
import math as m

import roslib; roslib.load_manifest('urdfdom_py')
import rospy
from urdf_parser_py.urdf import URDF

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from phand_core_lib.phand import *

class HandJointPublisher:

    def __init__(self, hand):

        self.phand = hand

        self.hand_prefix = "phand1_"

        robot_description = rospy.get_param("robot_description")
        robot_description = robot_description.replace("encoding=\"utf-8\"", "")
        self.robot = URDF.from_xml_string(robot_description)

        self.joint_state = JointState()

        self.joint_state.name = []

        self.generate_joint_list()
        self.joint_state.position = [0] * len(self.joint_state.name)

        self.publish_joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)

        self.l1 = 0
        self.l2 = 0

    def set_sensor_input(self, l1, l2):
        self.l1 = l1
        self.l2 = l2

    def generate_joint_list(self):

        for joint in self.robot.joints:
            if joint.type in ["revolute","prismatic"]:
                self.joint_state.name.append(joint.name)

    def update_joint_state(self):
        self.joint_state.header.stamp = rospy.Time.now()

        [theta1, theta2] = self.phand.jc.calculate_theta1_theta2(self.l1, self.l2)

        # print([self.l1, self.l2, theta1, theta2 ])

        # other joint mapping
        idx_theta1 = self.joint_state.name.index(self.hand_prefix +"rotation_x")
        idx_theta2 = self.joint_state.name.index(self.hand_prefix +"rotation_y")
        # theta1 = self.joint_state.position[idx_theta1]
        # theta2 = self.joint_state.position[idx_theta2]
        self.joint_state.position[idx_theta1] = theta1
        self.joint_state.position[idx_theta2] = theta2
        # Right cylinder
        idx_rcr = self.joint_state.name.index(self.hand_prefix +"rightcylinder_rod")
        idx_wbr = self.joint_state.name.index(self.hand_prefix +"wristBase_cylinderR")
        idx_hrvr = self.joint_state.name.index(self.hand_prefix +"horizontal_R_vertical_R")
        self.joint_state.position[idx_wbr] = -self.phand.jc.calculate_wristBase_cylinderR(theta1,theta2)
        self.joint_state.position[idx_hrvr] = -self.phand.jc.calculate_horizontal_R_vertical_R(theta1,theta2)
        self.joint_state.position[idx_rcr] = self.l2 #self.jc.calculate_l0() - self.jc.calculate_rigthcylinder_rod(theta1, theta2)

        # Left cylinder
        idx_lcr = self.joint_state.name.index(self.hand_prefix +"leftcylinder_rod")
        idx_wbl = self.joint_state.name.index(self.hand_prefix +"wristBase_horizontal_L")
        idx_hrvl = self.joint_state.name.index(self.hand_prefix +"horizontal_L_vertical_L")
        self.joint_state.position[idx_wbl] = self.phand.jc.calculate_wristBase_cylinderL(theta1,theta2)
        self.joint_state.position[idx_hrvl] = self.phand.jc.calculate_horizontal_L_vertical_L(theta1,theta2)
        self.joint_state.position[idx_lcr] = self.l1 # self.jc.calculate_leftcylinder_rod(theta1,theta2) - self.jc.calculate_l0()

        # Calculate the index joints
        idx_irod = self.joint_state.name.index(self.hand_prefix +"cylinder_rod")
        idx_idev = self.joint_state.name.index(self.hand_prefix +"index_deviation")
        idx_ibase = self.joint_state.name.index(self.hand_prefix +"index_cylinder_base")

        [phi1, phi2] = self.phand.jc.calculate_index_angles(self.joint_state.position[idx_irod])
        self.joint_state.position[idx_idev] = phi1
        self.joint_state.position[idx_ibase] =  phi2

        self.publish_joint_state.publish(self.joint_state)



    def find_joint_data(self, name):

        for joint_data in self.robot.joints:
            if joint_data.name == name:
                return joint_data


