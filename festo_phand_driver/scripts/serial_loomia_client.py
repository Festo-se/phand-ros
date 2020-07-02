#!/usr/bin/env python3
import rospy
from bionic_serial_client.bionic_serial_client import BionicSerialClient
from bionic_message_tools.bionic_message_tools import BionicMessageHandler
from bionic_messages.bionic_messages import BionicSetLoomiaValuesActionMessage

from bionic_messages.bionic_messages import BionicLoomiaMessage, BIONIC_MSG_IDS
from festo_phand_msgs.msg import GenericSensor
from festo_phand_msgs.srv import *
import copy

class BionicSoftHandSerialClientRosInterface:

    def __init__(self):

        self.loomia_pub = rospy.Publisher("festo/phand/connected_sensors/loomia_sensors", GenericSensor, queue_size=1)
        rospy.Service("festo/phand/loomia/set_configuration", LoomiaSensorConfig, self.loomia_config_srv_cb)

        msg_handler = BionicMessageHandler()
        msg = BionicLoomiaMessage(BIONIC_MSG_IDS.LOOMIA_BOARD)
        msg.register_cb(self.loomia_msg_cb)
        msg_handler.register_message_type(msg)

        self.action_msg = BionicSetLoomiaValuesActionMessage(2.2, [1000]*12, 75, 0, 0)
        msg_handler.register_message_type(self.action_msg)

        self.sc = BionicSerialClient(message_handler=msg_handler,
                                     baud=2000000)

    def loomia_msg_cb(self, msg: BionicLoomiaMessage):

        loomia_sensor = GenericSensor()
        loomia_sensor.name = msg.get_unique_name()
        loomia_sensor.id = msg.get_id()
        loomia_sensor.raw_values = copy.deepcopy(msg.pressures)
        loomia_sensor.raw_values.extend(msg.set_resitance_values)
        loomia_sensor.raw_values.append(msg.set_measurement_delay)
        loomia_sensor.raw_values.append(msg.set_ref_voltage)
        loomia_sensor.raw_values.append(msg.meas_ref_voltage)
        self.loomia_pub.publish(loomia_sensor)

    def loomia_config_srv_cb(self, msg: LoomiaSensorConfigRequest) -> LoomiaSensorConfigResponse:

        self.action_msg.set_data(msg.reference_voltage,
                                 msg.series_resistance,
                                 msg.d_column_switch,
                                 msg.led_logo,
                                 msg.led_board)

        return LoomiaSensorConfigResponse(True)


if __name__ == "__main__":
    rospy.init_node("serial_loomia_client")
    BionicSoftHandSerialClientRosInterface()
