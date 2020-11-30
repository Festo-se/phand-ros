#!/usr/bin/env python3

import matplotlib

from bionic_serial_client.bionic_serial_client import BionicSerialClient
from bionic_message_tools.bionic_message_tools import BionicMessageHandler
from phand_messages.loomia_messages import BionicLoomiaMessage, BionicSetLoomiaValuesActionMessage
from phand_messages.phand_message_constants import BIONIC_MSG_IDS

from festo_phand_msgs.srv import LoomiaSensorConfig, LoomiaSensorConfigRequest
from festo_phand_msgs.msg import GenericSensor

import copy
import logging
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import Animation
from matplotlib.widgets import Button
import time

class BionicSoftHandSerialClientRosInterface:

    def __init__(self):

        logging.basicConfig(level=logging.INFO)

        self._shutdown = False
        self.adc_reference_voltage = 2.2
        self.series_resistance_sensors = [1000]*11
        self.d_column_switch = 75
        self.logo_led = 0
        self.onboard_led = 0
        
        msg_handler = BionicMessageHandler()
        msg = BionicLoomiaMessage(BIONIC_MSG_IDS.LOOMIA_MSG_ID)
        msg.register_cb(self.loomia_msg_cb)
        msg_handler.register_message_type(msg)

        self.pressure_array = np.zeros(shape=[14,10])     
        self.init_plot()   

        rospy.Subscriber("festo/phand/connected_sensors/loomia_sensors", GenericSensor, self.loomia_msg_cb)
        self.loomia_cfg_srv = rospy.ServiceProxy("festo/phand/loomia/set_configuration", LoomiaSensorConfig)
        
        while not self._shutdown:
            self.update_plot()            
        
    def init_plot(self):

        self.fig, self.ax = plt.subplots()  
        ledboard = plt.axes([0.81, 0.05, 0.1, 0.075])
        plt.subplots_adjust(bottom=0.2)          
        bLedBoard = Button(ledboard, 'BOARD LED')
        bLedBoard.on_clicked(self.button_led_board_cb) 
        self.ax.imshow(self.pressure_array)                   

    def update_plot(self):

        self.ax.cla() 

        for x in range(0,14):
            for y in range(0,10):
                self.ax.text(y, x, self.pressure_array[x, y], ha="center", va="center", color="w")  

        ledboard = plt.axes([0.81, 0.05, 0.1, 0.075])
        plt.subplots_adjust(bottom=0.2)          
        bLedBoard = Button(ledboard, 'BOARD LED')
        bLedBoard.on_clicked(self.button_led_board_cb) 

        self.ax.imshow(self.pressure_array)
        plt.pause(0.01)        

    def button_led_board_cb(self, event):
        
        self.logo_led = not self.logo_led
        self.onboard_led = not self. onboard_led            
        print (f"MESSAGE SENT: {self.set_loomia_cfg()}")

    def loomia_msg_cb(self, msg):
        
        msg_length = len(msg.raw_values)
        
        for x in range(0,14):
            for y in range(0,10):
                index = (x * 10) + y 

                if index > 130:
                    self.pressure_array[x,y] = 0
                else:              
                    self.pressure_array[x,y] = msg.raw_values[index]     
        
    def set_loomia_cfg(self):
        
        msg = LoomiaSensorConfigRequest()
        msg.series_resistance = self.series_resistance_sensors
        msg.reference_voltage = self.adc_reference_voltage
        msg.d_column_switch = self.d_column_switch
        msg.led_logo = self.logo_led
        msg.led_board = self.onboard_led

        return self.loomia_cfg_srv(msg)

if __name__ == "__main__":
    rospy.init_node("test_loomia")
    BionicSoftHandSerialClientRosInterface()
