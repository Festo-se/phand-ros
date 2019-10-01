#!/usr/bin/env python3
import importlib
import json

__author__ = "Marinus Matthias Moerdijk"
__copyright__ = "Copyright 2019, Festo Coperate Bionic Projects"
__credits__ = ["Timo Schwarzer", "Marinus Matthias Moerdijk"]
__license__ = "GPL"
__version__ = "1.0.3"
__maintainer__ = "Marinus Moerdijk"
__email__ = "marinus.moerdijk@festo.com"
__status__ = "Experimental"


import rospy

from phand_core_lib.phand_driver import *
from festo_phand_msgs.msg import *
from festo_phand_msgs.srv import *
from bionic_messages.bionic_messages import *


class ROSPhandUdpDriver(PhandUdpDriver):
    """
    Wrapper class for the phand_core_lib to provide a ros interface for the phand
    """

    def __init__(self):
        super(ROSPhandUdpDriver, self).__init__()

        # Setup internal status
        self.hand_state = HandState()

        # Registercallback internal sensors
        self.messages["BionicAD7998Message"].register_cb(self.valve_terminal_cb)

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
        rospy.Subscriber("festo/phand/set_pressures", SimpleFluidPressures, callback=self.set_pressures_topic_cb,
                         queue_size=1)
        rospy.Subscriber("festo/phand/set_positions", Positions, callback=self.set_positions_topic_cb,
                         queue_size=1)

        # start the udp event loop
        importlib.reload(logging)
        self.run_in_thread()

        rate = rospy.Rate(50)
        rospy.loginfo("Starting ros event loop")
        while not rospy.is_shutdown():
            state_pub.publish(self.hand_state)
            rate.sleep()

        rospy.loginfo("Shutting down udp client")

        self.shutdown()

    def add_msg_id_to_state(self, msg):

        if not issubclass(type(msg), BionicMessageBase):
            return

        if msg.get_id() not in self.hand_state.connected_sensor_ids:
            self.hand_state.connected_sensor_ids.append(msg.get_id())
            self.hand_state.connected_sensor_names.append(msg.get_unique_name())

    def valve_terminal_cb(self, msg):
        """
        Callback for the BionicAD7998Message
        :param msg: Message from the udp connection of type BionicAD7998Message
        :return: none, updates internal state
        """
        self.add_msg_id_to_state(msg)
        self.hand_state.internal_sensors.pressure.values = msg.pressures
        self.hand_state.internal_sensors.valves.supply_valve_setpoints = msg.valve_setpoints[0:11]
        self.hand_state.internal_sensors.valves.exhaust_valve_setpoints = msg.valve_setpoints[12:24]

    def hand_bendsensor_cb(self, msg):

        self.add_msg_id_to_state(msg)

        bend_sensors = GenericSensor()
        bend_sensors.name = msg.get_unique_name()
        bend_sensors.id = msg.get_id()
        bend_sensors.state = msg.calibration_state
        bend_sensors.calibrated_values = msg.angles
        bend_sensors.raw_values = msg.raw_angles

        self.bendsensor_pub.publish(bend_sensors)

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

        self.send_data(BionicValveMsg(msg.supply_valve_setpoints, msg.exhaust_valve_setpoints ).data)


if __name__ == '__main__':
    ROSPhandUdpDriver()
