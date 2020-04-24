from enum import IntEnum
import rospy
from festo_phand_msgs.srv import *
from festo_phand_msgs.msg import *

class HAND_CTRL_MODES(IntEnum):
    VALVE_CONTROL     = 0
    PRESSURE_CONTROL  = 1
    POSITION_CONTROL  = 2

class Phand:

    def __init__(self):

        try:
            rospy.init_node("hand",anonymous=True)
        except Exception as e:
            pass

        self.simple_open_close = rospy.ServiceProxy("/festo/phand/close", SimpleOpenClose)
        self.set_config = rospy.ServiceProxy("/festo/phand/set_configuration", SetConfiguration)
        self.pressure_pup = rospy.Publisher("/festo/phand/set_pressures", SimpleFluidPressures, queue_size=1)
        self.pressures_msg = SimpleFluidPressures()
        self.pressures_msg.values = [0]*12

        self.simple_open_close_msg = SimpleOpenCloseRequest()
        self.configure_hand_msg = HandConfiguration()
        self.configure_hand_msg.grip_configuration = self.configure_hand_msg.CLAW
        self.configure_hand_msg.control_configuration = HAND_CTRL_MODES.PRESSURE_CONTROL

        self.config_msg = SetConfigurationRequest()
        self.config_msg.configuration = self.configure_hand_msg
        # self.set_config(self.config_msg)

        self.hand_state = HandState()
        rospy.Subscriber("festo/phand/state", HandState, self.hand_state_cb)

        rospy.sleep(1.0) # Let the node make a conntion to all subscribers

    def hand_state_cb(self, msg):
        self.hand_state = msg

    def set_control_mode(self, mode):

        self.configure_hand_msg.control_configuration = mode
        self.config_msg.configuration = self.configure_hand_msg
        self.set_config(self.config_msg)

    def grip_open(self):
        rospy.loginfo("hand open action")
        self.simple_open_close_msg.pressures = [0]*12
        self.simple_open_close_msg.pressures[0] = 500000.0
        self.simple_open_close(self.simple_open_close_msg)
        rospy.sleep(1)

    def grip_close(self):
        rospy.loginfo("hand close action")
        # close_hand_msg.configuration = configure_hand_msg
        # close_hand(close_hand_msg)

        self.pressures_msg.values = [100000.0, 405899.99999999994, 175000.0, 311850.0, 594000.0, 100000.0, 594000.0, 100000.0, 594000.0, 100000.0, 579150.0, 594000.0]
        self.pressure_pup.publish(self.pressures_msg)
        rospy.sleep(1)

    def set_pressures(self, pressures, time=0):
        start_pressures = self.hand_state.internal_sensors.actual_pressures.values
        rospy.loginfo("Current pressures : %s" % str(start_pressures))
        rospy.loginfo("Setting pressures to: %s" % str(pressures))
        if time == 0:
            self.pressures_msg.values = pressures
            self.pressure_pup.publish(self.pressures_msg)
        else:

            p_diff = []
            num_steps = int(time / 0.1)
            for ps, pe in zip(start_pressures, pressures):
                p_diff.append((ps-pe)/num_steps)

            rospy.loginfo("pdiff: %s" % str(p_diff))

            for step in range(num_steps):

                new_pressures = []
                for pd,pin in zip(p_diff, start_pressures):
                    new_pressures.append(pin-pd*step)

                self.pressures_msg.values = new_pressures
                rospy.loginfo("Setting pressures to: %i %s" % (step, str(self.pressures_msg.values)))
                self.pressure_pup.publish(self.pressures_msg)
                rospy.sleep(0.1)

            self.pressures_msg.values = pressures
            rospy.loginfo("Setting pressures to: %s" % ( str(self.pressures_msg.values)))
            self.pressure_pup.publish(self.pressures_msg)

        rospy.sleep(1.0)
