#!/usr/bin/env python3

__author__ = "Marinus Matthias Moerdijk & Timo Schwarzer"
__copyright__ = "Copyright 2020, Festo Coperate Bionic Projects"
__credits__ = ["Timo Schwarzer", "Marinus Matthias Moerdijk"]
__license__ = "GNU GPL v3.0"
__version__ = "1.0.6"
__maintainer__ = "Timo Schwarzer"
__email__ = "timo.schwarzer@festo.com"
__status__ = "Experimental"

# System imports
import importlib
import copy
import time
import logging
import threading

# Ros imports
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from diagnostic_msgs.msg import KeyValue

# Festo imports
from phand.phand import PHand
from phand.phand_constants import PHAND_STATE
from phand_messages.phand_message_constants import BIONIC_MSG_IDS
from phand_calibration.phand_sensor_calibration import PhandSensorCalibrationValue

from phand_messages.cylinder_messages import BionicCylinderSensorMessage
from phand_messages.flex_sensor_messages import BionicFlexSensorMessage
from phand_messages.imu_messages import BionicIMUDataMessage
from phand_messages.loomia_messages import BionicLoomiaMessage
from phand_messages.valve_terminal_messages import BionicValveMessage

from festo_phand_joint_publisher import HandJointPublisher
from festo_phand_msgs.msg import *
from festo_phand_msgs.srv import *

class ROSPhandUdpDriver:
    """
    Wrapper class for the phand_core_lib to provide a ros interface for the phand
    """

    # required_msgs_ids = [BIONIC_MSG_IDS.VALVE_MODULE_MSG_ID,                         
    #                      BIONIC_MSG_IDS.IMU_MAINBOARD_MSG_ID,
    #                      BIONIC_MSG_IDS.LOOMIA_MSG_ID,
    #                      BIONIC_MSG_IDS.FLEX_SENSOR_MSG_ID,
    #                      BIONIC_MSG_IDS.CYLINDER_SENSOR_MSG_ID
    #                      ]

    required_msgs_ids = [BIONIC_MSG_IDS.VALVE_MODULE_MSG_ID,                         
                         BIONIC_MSG_IDS.FLEX_SENSOR_MSG_ID,
                         BIONIC_MSG_IDS.CYLINDER_SENSOR_MSG_ID
                         ]

    def __init__(self):

        # Init ros
        rospy.init_node("festo_phand_driver")

        # start the udp event loop
        importlib.reload(logging)
        logging.basicConfig(format='%(asctime)s,%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
                            datefmt='%Y-%m-%d:%H:%M:%S',
                            level=logging.DEBUG)

        self.phand = PHand()

        self.joinpub = []
        self.joint_pup = False

        if rospy.has_param("robot_description"):
            self.joinpub = HandJointPublisher(self.phand)
            self.joint_pup = True

        self.phand.register_new_data_available_cb(self.new_data_available_cb)
        self.phand.set_required_msg_ids(self.required_msgs_ids)

        # Setup internal status
        self.hand_state = HandState()
        self.hand_state.state = -1

        self.current_time = 0

        state_pub = rospy.Publisher("festo/phand/state", HandState, queue_size=1)
        
        self.flex_pub = rospy.Publisher("festo/phand/connected_sensors/flex_sensors", GenericSensor, queue_size=1)
        self.loomia_pub = rospy.Publisher("festo/phand/connected_sensors/loomia_sensors", GenericSensor, queue_size=1)
        self.cylinder_pub = rospy.Publisher("festo/phand/connected_sensors/cylinder_sensors", GenericSensor, queue_size=1)

        # Subscribe to topics
        rospy.Subscriber("festo/phand/set_valve_setpoints", ValveSetPoints, callback=self.set_valves_topic_cb, queue_size=1)
        rospy.Subscriber("festo/phand/set_pressures", SimpleFluidPressures, callback=self.set_pressures_topic_cb, queue_size=1)
        
        rospy.Subscriber("festo/phand/fingers/set_positions", Positions, callback=self.set_finger_positions_cb, queue_size=1)
        rospy.Subscriber("festo/phand/wrist/set_positions", Positions, callback=self.set_wrist_position_cb, queue_size=1)

        # Offer services
        rospy.Service("festo/phand/close", SimpleOpenClose, self.simple_close_cb)
        rospy.Service("festo/phand/open", SimpleOpenClose, self.simple_open_cb)
        rospy.Service("festo/phand/set_configuration", SetConfiguration, self.set_configuration_cb)
        rospy.Service("festo/phand/loomia/set_configuration", LoomiaSensorConfig, self.loomia_config_srv_cb)
        rospy.Service("festo/phand/flexsensors/set_configuration", FlexSensorConfig, self.flexsensor_config_srv_cb)

        rospy.Service("festo/phand/set_calibration", SetHandCalibration, self.set_calibration_cb)
       
        self.pressures = [100000.0] * 12         

        # Initialize the wrist controller
        #                 
        rate = rospy.Rate(100)
        rospy.loginfo("Starting ros event loop")
        while not rospy.is_shutdown(): 

            # Generate the hand state message
            self.generate_hand_state()

            # Publish the gernerated hand state
            state_pub.publish(self.hand_state)

            # If the hand robot is started, publish the joint values
            if self.joint_pup:
                self.joinpub.update_joint_state()
            
            rate.sleep()

        rospy.loginfo("Shutting down udp client")
        self.phand.shutdown()

    def new_data_available_cb(self):
        """
        This is called when new data is available from the hand.
        """ 

        press_diff = self.phand.messages["BionicValveMessage"].last_msg_received_time - self.current_time 
        imu_diff = self.phand.messages["BionicIMUDataMessage"].last_msg_received_time - self.current_time        
        loomia_diff = self.phand.messages["BionicLoomiaMessage"].last_msg_received_time - self.current_time
        flex_diff = self.phand.messages["BionicFlexMessage"].last_msg_received_time - self.current_time
        cylinder_diff = self.phand.messages["BionicCylinderSensorMessage"].last_msg_received_time - self.current_time

        if press_diff >= 0:
            self.valve_terminal_generate(self.phand.messages["BionicValveMessage"])
        
        if imu_diff >= 0:            
            self.internal_imu_generate(self.phand.messages["BionicIMUDataMessage"])

        if loomia_diff >= 0:
            self.hand_loomia_generate(self.phand.messages["BionicLoomiaMessage"])

        if flex_diff >= 0:
            self.hand_flex_generate(self.phand.messages["BionicFlexMessage"])

        if cylinder_diff >= 0:
            self.hand_cylinder_generate(self.phand.messages["BionicCylinderSensorMessage"])

        self.current_time = int(round(time.time() * 1000))

    def generate_hand_state(self):
        """
        Generate the state of the hand based on the communication state and the number of sensors connected

        :return:
        """

        self.hand_state.status_codes = []
        self.hand_state.hand_id = str(self.phand.hand_id)
        
        self.hand_state.state = self.phand.com_state
       
        self.hand_state.connected_sensor_ids = self.phand.connected_sensor_ids
        self.hand_state.connected_sensor_names = self.phand.connected_sensor_names

        for key_value in self.phand.status_codes:            
            state = KeyValue()
            state.key = key_value[0]
            state.value = key_value[1]            
            self.hand_state.status_codes.append(state)
        
    # Service callbacks
    def set_calibration_cb(self, msg:SetHandCalibrationRequest):
        
        calib_data = []
        for calib_value in msg.calibration_data:
            calib_data.append(PhandSensorCalibrationValue(value_id=calib_value.id, value=calib_value.values))
        
        self.phand.set_calibration(calib_data)

        return SetHandCalibrationResponse(True)

    def flexsensor_config_srv_cb(self, msg: FlexSensorConfigRequest):

        if len(msg.series_resistance_top) != 7 or len(msg.series_resistance_bottom) != 7:
            rospy.logerr("Number of series resistance does not match the required number of 7")
            return FlexSensorConfigResponse(False)

        self.phand.set_flexsensor_config(msg.led_green,
                                         msg.led_blue,
                                         msg.led_red,
                                         msg.override_leds,
                                         msg.series_resistance_top,
                                         msg.series_resistance_bottom)

        return FlexSensorConfigResponse(True)

    def loomia_config_srv_cb(self, msg:LoomiaSensorConfigRequest):

        if len(msg.series_resistance) != 11:
            rospy.logerr("Number of series resistance does not match the required number of 11")
            return LoomiaSensorConfigResponse(False)

        self.phand.set_loomia_config(msg.reference_voltage,
                                     msg.series_resistance,
                                     msg.d_column_switch,
                                     msg.led_logo,
                                     msg.led_board)

        return LoomiaSensorConfigResponse(True)

    def set_configuration_cb(self, msg):
        """
        Callback function for the set configuration service
        :param msg: SetConfigurationRequest
        :return: SetConfigurationReply
        """

        resp = SetConfigurationResponse()

        grip_config = self.phand.set_grip_config(msg.configuration.grip_configuration)
        ctrl_config = self.phand.set_ctrl_mode(msg.configuration.control_configuration)

        if grip_config and ctrl_config:
            
            resp.success = True
            resp.state.key = "0"
            resp.state.value = "Configuration set"

        elif grip_config and not ctrl_config:
            
            resp.success = True
            resp.state.key = "1"
            resp.state.value = "Grip configuration set"
        
        elif ctrl_config and not grip_config:

            resp.success = True
            resp.state.key = "2"
            resp.state.value = "Control configuration set"

        else:

            resp.success = False
            resp.state.key = "3"
            resp.state.value = "Problem with setting the configuration"
        
        return resp

    def simple_open_cb(self, msg):
        """
        Callback function for the simple open function.        
        :param msg: SimpleOpenRequest
        :return: SimpleOpenResult
        """

        resp = SimpleOpenCloseResponse()

        if self.phand.simple_open(msg.speed, msg.pressures):

            resp.success = True      
            resp.state.key = "0"
            resp.state.value = "Executes a simple open"

        else:

            rospy.loginfo("Could not open the hand.")
            resp.success = False      
            resp.state.key = "1"
            resp.state.value = "Could not execute a simple open"
            
        # Return result                  
        return resp

    def simple_close_cb(self, msg):
        """
        Callback function for the simple close function
        :param msg: SimpleCloseRequest
        :return: SimpleCloseResult
        """

        resp = SimpleOpenCloseResponse()

        if self.phand.simple_close(msg.speed, msg.pressures):
            
            resp.success = True
            resp.state.key = "0"
            resp.state.value = "Execute a simple close"

        else:
            
            rospy.loginfo("Could not close the hand.")
            resp.success = False      
            resp.state.key = "1"
            resp.state.value = "Could not execute a simple close"
            
        # Return result                  
        return resp       

    # Topic Callbacks
    def set_wrist_position_cb(self, msg):
        """
        Set the wrist position according to the mm input values
        """
        
        self.phand.set_wrist_position_data(msg.positions)

    def set_finger_positions_cb(self, msg):
        """def wristController(self):
        If the position control is activated, set the positions of the finger.
        """

        self.phand.set_finger_position_data(msg.positions)

    def set_pressures_topic_cb(self, msg):
        """
        Set the pressure for the valves
        Range: 100000.0 - 600000.0 psi
        """

        for i in range(len(msg.values)):
            self.pressures[i] = msg.values[i]

        self.phand.set_pressure_data(self.pressures)             

    def set_valves_topic_cb(self, msg):
        """
        Set the valves directly
        """
        
        self.phand.set_valve_opening_data(msg.supply_valve_setpoints, msg.exhaust_valve_setpoints)        
    
    def valve_terminal_generate(self, msg):
        """
        Callback for the BionicValveMessage
        :param msg: Message from the udp connection of type BionicValveMessage
        :return: none, updates internal state
        """

        self.hand_state.mode.mode = self.phand.ctrl_mode
        self.hand_state.internal_sensors.actual_pressures.values = msg.actual_pressures
        self.hand_state.internal_sensors.set_pressures.values = msg.set_pressures
        self.hand_state.internal_sensors.valves.supply_valve_setpoints = msg.valve_setpoints[0:12]
        self.hand_state.internal_sensors.valves.exhaust_valve_setpoints = msg.valve_setpoints[12:24]

    def internal_imu_generate(self, msg):
        """
        Generate the imu sensor msg
        """

        self.hand_state.internal_sensors.imu.linear_acceleration.x = msg.acc_x
        self.hand_state.internal_sensors.imu.linear_acceleration.y = msg.acc_y
        self.hand_state.internal_sensors.imu.linear_acceleration.z = msg.acc_z
        self.hand_state.internal_sensors.imu.angular_velocity.x = msg.gyro_x
        self.hand_state.internal_sensors.imu.angular_velocity.y = msg.gyro_y
        self.hand_state.internal_sensors.imu.angular_velocity.z = msg.gyro_z
        self.hand_state.internal_sensors.imu.orientation.x = msg.quat_x
        self.hand_state.internal_sensors.imu.orientation.y = msg.quat_y
        self.hand_state.internal_sensors.imu.orientation.z = msg.quat_z
        self.hand_state.internal_sensors.imu.orientation.w = msg.quat_w
        self.hand_state.internal_sensors.mag.magnetic_field.x = msg.mag_x
        self.hand_state.internal_sensors.mag.magnetic_field.y = msg.mag_y
        self.hand_state.internal_sensors.mag.magnetic_field.z = msg.mag_z

    def hand_loomia_generate(self, msg: BionicLoomiaMessage):
        """
        Generate the loomia sensor msg
        """

        loomia_sensor = GenericSensor()
        loomia_sensor.name = msg.get_unique_name()
        loomia_sensor.id = msg.get_id()

        loomia_sensor.raw_values = copy.deepcopy(msg.pressures)
        loomia_sensor.raw_values.extend(msg.set_resitance_values)
        loomia_sensor.raw_values.append(msg.set_measurement_delay)
        loomia_sensor.raw_values.append(msg.set_ref_voltage)
        loomia_sensor.raw_values.append(msg.meas_ref_voltage)

        # Calculating the real voltage values for the
        # raw using the method from RM0038 Rev. 287/906 (Reference manual st)
        for value in msg.pressures:
            calibrated_value = (msg.meas_ref_voltage / pow(2, 12))*value
            loomia_sensor.calibrated_values.append(calibrated_value)

        self.loomia_pub.publish(loomia_sensor)

    def hand_cylinder_generate(self, msg):
        """
        Generate the cylinder sensor message
        """

        cylinder_sensor = GenericSensor()
        cylinder_sensor.name = msg.get_unique_name()
        cylinder_sensor.provides = msg.provides
        cylinder_sensor.id = msg.get_id()
        cylinder_sensor.raw_values = msg.values
        cylinder_sensor.calibrated_values = msg.calibrated_values

        if self.joint_pup:
            self.joinpub.set_sensor_input(l1=msg.calibrated_values[1],
                                          l2=msg.calibrated_values[2],
                                          lrod=msg.calibrated_values[0]
                                          )

        self.cylinder_pub.publish(cylinder_sensor)             

    def hand_flex_generate(self, msg):
        """
        Generate the flex sensor msg
        """

        flex_sensor = GenericSensor()
        flex_sensor.name = msg.get_unique_name()
        flex_sensor.id = msg.get_id()
        
        flex_sensor.raw_values = [0] * 11

        flex_sensor.raw_values[0] = msg.top_sensors[0]
        flex_sensor.raw_values[1] = msg.top_sensors[1]
        flex_sensor.raw_values[2] = msg.top_sensors[2]
        flex_sensor.raw_values[3] = msg.top_sensors[3]
        flex_sensor.raw_values[4] = msg.top_sensors[4]

        flex_sensor.raw_values[5] = msg.bot_sensors[0]
        flex_sensor.raw_values[6] = msg.bot_sensors[1]
        flex_sensor.raw_values[7] = msg.bot_sensors[2]
        flex_sensor.raw_values[8] = msg.bot_sensors[3]
        flex_sensor.raw_values[9] = msg.bot_sensors[4]

        flex_sensor.raw_values[10] = msg.drvs_potti

        self.flex_pub.publish(flex_sensor)

if __name__ == '__main__':
    ROSPhandUdpDriver()
