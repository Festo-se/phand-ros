#!/usr/bin/env python3

import rospy

from festo_phand_msgs.msg import GenericSensor

from bionic_pid_v2_control.motion_control import MotionControl

import numpy as np
import math
import time
import threading

class SoftHandWristController:
    ''' SoftHand Wrist Controller to control the wrist position based on pressure outputs '''
    def __init__(self):

        # Position vector of the wrist
        self.wrist = np.zeros(2)
        self.index = np.zeros(1)

        # Wrist desired positPIDion
        self.wrist_des = np.zeros(2)
       
        # Initialize update rate of 100hz 
        #self.rate = rospy.Rate(100)

        self.wrist_ctrl = MotionControl(2500000, 49000000, 1500000, self.wrist_des)

        self.main_loop_thread = threading.Thread(target=self.start)
        self.main_loop_thread.start()
                
    def start(self):
        
        # Subscriber            
        rospy.wait_for_message("/festo/phand/connected_sensors/cylinder_sensors", GenericSensor)
        rospy.Subscriber("/festo/phand/connected_sensors/cylinder_sensors", GenericSensor, self.handCylinderCallback)    
        
        while not rospy.is_shutdown():
            rospy.sleep(10)

    def handCylinderCallback(self, message):

        self.index[0] = message.raw_values[0]
        self.wrist[0] = message.calibrated_values[1] # left -0.009 .. 0.015  (do calibration again)
        self.wrist[1] = message.calibrated_values[2] # right -0.009 .. 0.015 (do calibration again)        

    def wristController(self):
        ''' Move wrist based on position control 
            the ambient pressure value is 100000 psi equal to 0 bar
            the maximum pressure value is 600000 psi equal to 5 bar '''       
        
        # Update wrist controller
        self.wrist_p = self.wrist_ctrl.update(self.wrist, self.wrist_des)

        #print(f"IST: {self.wrist} SOLL: {self.wrist_des} UPDATE: {self.wrist_p}")

        return self.wrist_p        

    def move_wrist(self, wirst_left_des, wrist_right_des):
        self.wrist_des = np.array([wirst_left_des, wrist_right_des])        
    