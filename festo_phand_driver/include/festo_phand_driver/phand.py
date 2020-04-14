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

        self.close_hand = rospy.ServiceProxy("/festo/phand/close", SimpleClose)
        self.open_hand = rospy.ServiceProxy("/festo/phand/open", SimpleOpen)
        self.set_config = rospy.ServiceProxy("/festo/phand/set_configuration", SetConfiguration)
        self.pressure_pup = rospy.Publisher("/festo/phand/set_pressures", SimpleFluidPressures, queue_size=1)
        self.pressures_msg = SimpleFluidPressures()

        self.close_hand_msg = SimpleCloseRequest()
        self.open_hand_msg = SimpleOpenRequest()
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
        self.open_hand(self.open_hand_msg)
        rospy.sleep(1)

    def grip_close(self):
        rospy.loginfo("hand close action")
        # close_hand_msg.configuration = configure_hand_msg
        # close_hand(close_hand_msg)
        self.pressures_msg.values[0] = 0.0 * 10e5
        self.pressures_msg.values[1] = 3.7 * 10e5
        self.pressures_msg.values[2] = 0.0 * 10e5
        self.pressures_msg.values[3] = 1.0 * 10e5
        self.pressures_msg.values[4] = 5.3 * 10e5
        self.pressures_msg.values[5] = 0.0 * 10e5
        self.pressures_msg.values[6] = 4.6 * 10e5
        self.pressures_msg.values[7] = 0.0 * 10e5
        self.pressures_msg.values[8] = 4.3 * 10e5
        self.pressures_msg.values[9] = 1.2 * 10e5
        self.pressures_msg.values[10] = 2.7 * 10e5
        self.pressures_msg.values[11] = 3.0 * 10e5

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

            for step in xrange(num_steps):

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