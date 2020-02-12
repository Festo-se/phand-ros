#!/usr/bin/env python3
__author__ = "Marinus Matthias Moerdijk"
__copyright__ = "Copyright 2019, Festo Coperate Bionic Projects"
__credits__ = ["Marinus Matthias Moerdijk"]
__license__ = "GPL"
__version__ = "1.0.3"
__maintainer__ = "Marinus Moerdijk"
__email__ = "marinus.moerdijk@festo.com"
__status__ = "Experimental"

# System imports
import importlib
# Ros imports
import rospy
from diagnostic_msgs.msg import KeyValue
# Festo imports
from phand_core_lib.phand import PHand, PHandState
from festo_phand_msgs.msg import *
from festo_phand_msgs.srv import *
from bionic_messages.bionic_messages import *


class ROSPhandUdpDriver():
    """
    Wrapper class for the phand_core_lib to provide a ros interface for the phand
    """

    required_msgs_ids = [BIONIC_MSG_IDS.VALVE_MODULE,                         
                         BIONIC_MSG_IDS.IMU_MAINBOARD,
                         BIONIC_MSG_IDS.LOOMIA_BOARD,
                         BIONIC_MSG_IDS.FLEX_BOARD,
                         BIONIC_MSG_IDS.CYLINDER_SENSOR
                         ]

    def __init__(self):

        self.phand = PHand()        
        self.phand.register_new_data_available_cb(self.new_data_available_cb)
        self.phand.set_required_msg_ids(self.required_msgs_ids)
        
        # Setup internal status
        self.hand_state = HandState()
        self.hand_state.state = -1

        self.current_time = 0

        # Init ros
        rospy.init_node("festo_phand_driver")
        state_pub = rospy.Publisher("festo/phand/state", HandState, queue_size=1)
        
        self.flex_pub = rospy.Publisher("festo/phand/connected_sensors/flex_sensors", GenericSensor, queue_size=1)
        self.loomia_pub = rospy.Publisher("festo/phand/connected_sensors/loomia_sensors", GenericSensor, queue_size=1)
        self.cylinder_pub = rospy.Publisher("festo/phand/connected_sensors/cylinder_sensors", GenericSensor, queue_size=1)

        # Subscribe to topics
        rospy.Subscriber("festo/phand/set_valve_setpoints", ValveSetPoints, callback=self.set_valves_topic_cb, queue_size=1)
        rospy.Subscriber("festo/phand/set_pressures", SimpleFluidPressures, callback=self.set_pressures_topic_cb, queue_size=1)
        rospy.Subscriber("festo/phand/set_positions", Positions, callback=self.set_positions_topic_cb, queue_size=1)

        # Offer services
        rospy.Service("festo/phand/close", SimpleClose, self.simple_close_cb)
        rospy.Service("festo/phand/open", SimpleOpen, self.simple_open_cb)
        rospy.Service("festo/phand/set_configuration", SetConfiguration, self.set_configuration_cb)        

        # start the udp event loop
        importlib.reload(logging)        

        rate = rospy.Rate(100)
        rospy.loginfo("Starting ros event loop")
        while not rospy.is_shutdown():
            self.generate_hand_state()
            state_pub.publish(self.hand_state)
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

    def set_configuration_cb(self, msg):
        """
        Callback function for the set configuration service
        :param msg: SetConfigurationRequest
        :return: SetConfigurationReply
        """

        resp = SetConfigurationResponse()

        grip_config = self.phand.set_grip_config_pressure(msg.configuration.grip_configuration)
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

    def simple_open_cb(self, msg: SimpleOpenRequest):
        """
        Callback function for the simple open function.        
        :param msg: SimpleOpenRequest
        :return: SimpleOpenResult
        """

        resp = SimpleOpenResponse()

        if self.phand.simple_open_pressure():

            rospy.loginfo("Opening the hand.")
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

    def simple_close_cb(self, msg: SimpleCloseRequest):
        """
        Callback function for the simple close function
        :param msg: SimpleCloseRequest
        :return: SimpleCloseResult
        """

        resp = SimpleCloseResponse()        

        if self.phand.simple_close():
            
            rospy.loginfo("Closing the hand in the grip mode: %s", (self.phand.grip_mode))
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
    def set_positions_topic_cb(self, msg):
        """
        If the position control is activated, set the positions of the finger.
        """

        self.phand.set_position_data(msg.positions)

    def set_pressures_topic_cb(self, msg):
        """
        Set the pressure for the valves
        Range: 100000.0 - 400000.0 psi
        """

        # values = [0] * 12
        # for x in range(len(msg.values)):
        #     values[x] = msg.values[x] * 100000.0 + 100000.0  

        # print(values)

        self.phand.set_pressure_data(msg.values)

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
        self.hand_state.internal_sensors.valves.supply_valve_setpoints = msg.valve_setpoints[0:11]
        self.hand_state.internal_sensors.valves.exhaust_valve_setpoints = msg.valve_setpoints[12:24]
    
    def hand_bendsensor_generate(self, msg):

        bend_sensors = GenericSensor()
        bend_sensors.name = msg.get_unique_name()
        bend_sensors.id = msg.get_id()
        bend_sensors.state = msg.calibration_state
        bend_sensors.calibrated_values = msg.angles
        bend_sensors.raw_values = msg.raw_angles

        self.flex_pub.publish(bend_sensors)

    def internal_imu_generate(self, msg):

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

    def hand_loomia_generate(self, msg):

        loomia_sensor = GenericSensor()
        loomia_sensor.name = msg.get_unique_name()
        loomia_sensor.id = msg.get_id()
        loomia_sensor.raw_values = msg.pressures

        self.loomia_pub.publish(loomia_sensor) 

    def hand_cylinder_generate(self, msg):
        
        cylinder_sensor = GenericSensor()
        cylinder_sensor.name = msg.get_unique_name()
        cylinder_sensor.id = msg.get_id()
        cylinder_sensor.raw_values = msg.values

        self.cylinder_pub.publish(cylinder_sensor)             

    def hand_flex_generate(self, msg):

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
