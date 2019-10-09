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
import functools
# Ros imports
import rospy
from diagnostic_msgs.msg import KeyValue
# Festo imports
from phand_core_lib.phand_driver import *
from festo_phand_msgs.msg import *
from festo_phand_msgs.srv import *
from bionic_messages.bionic_messages import *


class ROSPhandUdpDriver(PhandUdpDriver):
    """
    Wrapper class for the phand_core_lib to provide a ros interface for the phand
    """

    required_msgs_ids = [BIONIC_MSG_IDS.VALVE_MODULE,
                         BIONIC_MSG_IDS.SPECTRA_SENSOR,
                         BIONIC_MSG_IDS.IMU_MAINBOARD
                         ]
    hand_states = ["OFFLINE", "ONLINE", "ERROR"]

    def __init__(self):

        PhandUdpDriver.__init__(self)

        # Setup internal status
        self.hand_state = HandState()
        self.hand_state.state = -1

        # Registercallback internal sensors
        self.messages["BionicAD7998Message"].register_cb(self.valve_terminal_cb)
        self.messages["InternalIMUDMessage"].register_cb(self.internal_imu_cb)

        # Registercall back external sensors
        self.messages["BionicSpectreMessage"].register_cb(self.hand_bendsensor_cb)

        # Init ros
        rospy.init_node("festo_phand_driver")
        state_pub = rospy.Publisher("festo/phand/state", HandState, queue_size=1)

        bend_sensors = GenericSensor()
        self.bendsensor_pub = rospy.Publisher("festo/phand/connected_sensors/bend_sensors", GenericSensor, queue_size=1)

        # Subscribe to topics
        rospy.Subscriber("festo/phand/set_valve_setpoints", ValveSetPoints, callback=self.set_valves_topic_cb,
                         queue_size=1)
        # rospy.Subscriber("festo/phand/set_pressures", SimpleFluidPressures, callback=self.set_pressures_topic_cb,
        #                  queue_size=1)
        # rospy.Subscriber("festo/phand/set_positions", Positions, callback=self.set_positions_topic_cb,
        #                  queue_size=1)

        # Offer services
        rospy.Service("festo/phand/close", SimpleClose, self.simple_close_cb)
        rospy.Service("festo/phand/open", SimpleOpen, self.simple_open_cb)
        rospy.Service("festo/phand/set_configuration", SetConfiguration, self.set_configuration_cb)

        # start the udp event loop
        importlib.reload(logging)
        self.run_in_thread()

        rate = rospy.Rate(100)
        rospy.loginfo("Starting ros event loop")
        while not rospy.is_shutdown():
            self.generate_hand_state()
            state_pub.publish(self.hand_state)
            rate.sleep()

        rospy.loginfo("Shutting down udp client")

        self.shutdown()

    def generate_hand_state(self):
        """
        Generate the state of the hand based on the communication state and the number of sensors connected

        :return:
        """

        self.hand_state.status_codes = []

        old_hand_state = self.hand_state.state

        if self.state in ['ERROR']:
            self.hand_state.state = HandState.ERROR
            state = KeyValue()
            state.key = "1"
            state.value = "No connection with the hand possible. Do you have a ip in the same subnet as the hand?"
            self.hand_state.status_codes.append(state)
            return

        if self.state not in ['CONNECTED']:
            self.hand_state.state = HandState.OFFLINE
            state = KeyValue()
            state.key = "2"
            state.value = "Internal state: " + self.state
            self.hand_state.status_codes.append(state)
            return

        if self.state in ['CONNECTED']:
            self.hand_state.state = HandState.ONLINE
            state = KeyValue()
            state.key = "3"
            state.value = "Internal state: " + self.state
            self.hand_state.status_codes.append(state)

        if not all(elem in self.hand_state.connected_sensor_ids for elem in self.required_msgs_ids):
            self.hand_state.state = HandState.ERROR
            state = KeyValue()
            state.key = "4"
            state.value = "Not all required sensors are available"
            self.hand_state.status_codes.append(state)

        if self.hand_state.state != old_hand_state:
            rospy.loginfo("Hand state switched to: " + self.hand_states[self.hand_state.state])

    # Service callbacks

    def set_configuration_cb(self, msg):
        """
        Callback function for the set configuration service
        :param msg: SetConfigurationRequest
        :return: SetConfigurationReply
        """
        resp = SetConfigurationResponse()
        resp.success = False
        resp.state.key = "1"
        resp.state.value = "Not yet implemented"
        return resp

    def simple_open_cb(self, msg: SimpleOpenRequest):
        """
        Callback function for the simple open function
        :param msg: SimpleOpenRequest
        :return: SimpleOpenResult
        """

        # Set all finger pressures to 0
        setpoints = BionicValveMsg(self.hand_state.internal_sensors.valves.supply_valve_setpoints,
                                   self.hand_state.internal_sensors.valves.exhaust_valve_setpoints)
        setpoints.supply_valve_setpoints[0:10] = [0] * 10
        setpoints.exhaust_valve_setpoints[0:10] = [1.0] * 10
        self.send_data(setpoints.data)
        # Wait for all pressures to drop to 0
        # Return result
        resp = SimpleOpenResponse()
        resp.success = True
        return resp

    def simple_close_cb(self, msg: SimpleCloseRequest):
        """
        Callback function for the simple close function
        :param msg: SimpleCloseRequest
        :return: SimpleCloseResult
        """

        opening = min( max( 0.3 + msg.speed*0.7 , 0.0), 1.0)

        setpoints = BionicValveMsg(self.hand_state.internal_sensors.valves.supply_valve_setpoints,
                                   self.hand_state.internal_sensors.valves.exhaust_valve_setpoints)
        setpoints.supply_valve_setpoints[0:10] = [opening] * 10
        setpoints.exhaust_valve_setpoints[0:10] = [0.0] * 10
        self.send_data(setpoints.data)
        # Set hand to required configuration
        # Set all pressures to the max pressure
        # Check if pressure is reached

        resp = SimpleCloseResponse()
        resp.success = True
        resp.state.key = "1"
        resp.state.value = "Dummy implementation until pressure control is available"
        return resp

    # Topic Callbacks

    def set_positions_topic_cb(self, msg):
        msg = BionicActionMessage(sensor_id=BIONIC_MSG_IDS.VALVE_MODULE,
                                  action_id=VALVE_ACTION_IDS.POSITION,
                                  action_values=msg.pressures
                                  )
        self.send_data(msg)

    def set_pressures_topic_cb(self, msg):

        msg = BionicActionMessage(sensor_id=BIONIC_MSG_IDS.VALVE_MODULE,
                                  action_id=VALVE_ACTION_IDS.PRESSURE,
                                  action_values=msg.pressures
                                  )
        self.send_data(msg)

    def set_valves_topic_cb(self, msg):

        self.send_data(BionicValveMsg(msg.supply_valve_setpoints, msg.exhaust_valve_setpoints).data)

    # Udp message callbacks
    def add_msg_id_to_state(func):
        """
        Decorator function to add message id from message
        :return:
        """

        @functools.wraps(func)
        def wrap(self, *args, **kwargs):
            msg = args[0]

            if not issubclass(type(msg), BionicMessageBase):
                return

            if msg.get_id() not in self.hand_state.connected_sensor_ids:
                self.hand_state.connected_sensor_ids.append(msg.get_id())
                self.hand_state.connected_sensor_names.append(msg.get_unique_name())

            return func(self, *args, **kwargs)

        return wrap

    @add_msg_id_to_state
    def valve_terminal_cb(self, msg):
        """
        Callback for the BionicAD7998Message
        :param msg: Message from the udp connection of type BionicAD7998Message
        :return: none, updates internal state
        """

        self.hand_state.internal_sensors.pressure.values = msg.pressures
        self.hand_state.internal_sensors.valves.supply_valve_setpoints = msg.valve_setpoints[0:11]
        self.hand_state.internal_sensors.valves.exhaust_valve_setpoints = msg.valve_setpoints[12:24]

    @add_msg_id_to_state
    def hand_bendsensor_cb(self, msg):

        bend_sensors = GenericSensor()
        bend_sensors.name = msg.get_unique_name()
        bend_sensors.id = msg.get_id()
        bend_sensors.state = msg.calibration_state
        bend_sensors.calibrated_values = msg.angles
        bend_sensors.raw_values = msg.raw_angles

        self.bendsensor_pub.publish(bend_sensors)

    @add_msg_id_to_state
    def internal_imu_cb(self, msg):

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


if __name__ == '__main__':
    ROSPhandUdpDriver()
