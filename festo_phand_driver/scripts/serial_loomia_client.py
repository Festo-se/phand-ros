#!/usr/bin/env python3
import rospy
from bionic_serial_client.bionic_serial_client import BionicSerialClient
from bionic_message_tools.bionic_message_tools import BionicMessageHandler
from bionic_messages.bionic_messages import BionicSetLoomiaValuesActionMessage

from bionic_messages.bionic_messages import BionicLoomiaMessage, BIONIC_MSG_IDS
from festo_phand_msgs.msg import GenericSensor
from festo_phand_msgs.srv import *

class BionicSoftHandSerialClientRosInterface:

    def __init__(self):

        self.loomia_pub = rospy.Publisher("festo/phand/connected_sensors/loomia_sensors", GenericSensor, queue_size=1)
        rospy.Service("festo/phand/set_loomia_configuration", LoomiaSensorConfig, self.loomia_config_srv_cb)

        msg_handler = BionicMessageHandler()
        msg = BionicLoomiaMessage(BIONIC_MSG_IDS.LOOMIA_BOARD)
        msg.register_cb(self.loomia_msg_cb)
        msg_handler.register_message_type(msg)

        self.action_msg = BionicSetLoomiaValuesActionMessage(2.2, [1000]*12, 75)
        msg_handler.register_message_type(self.action_msg)

        self.sc = BionicSerialClient(message_handler=msg_handler,
                                     baud=2000000)

    def loomia_msg_cb(self, msg: BionicLoomiaMessage):

        loomia_sensor = GenericSensor()
        loomia_sensor.name = msg.get_unique_name()
        loomia_sensor.id = msg.get_id()
        loomia_sensor.raw_values = msg.pressures
        self.loomia_pub.publish(loomia_sensor)

    def loomia_config_srv_cb(self, msg: LoomiaSensorConfigRequest) -> LoomiaSensorConfigResponse:

        self.action_msg.set_data(msg.reference_voltage,
                                 msg.series_resistance,
                                 msg.d_column_switch)

        return LoomiaSensorConfigResponse(True)


if __name__ == "__main__":
    rospy.init_node("serial_loomia_client")
    BionicSoftHandSerialClientRosInterface()
