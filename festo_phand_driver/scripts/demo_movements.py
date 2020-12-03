#!/usr/bin/env python3

__author__ = "Timo Schwarzer"
__copyright__ = "Copyright 2020, Festo Coperate Bionic Projects"
__credits__ = ["Timo Schwarzer"]
__license__ = "GNU GPL v3.0"
__version__ = "1.0.5"
__maintainer__ = "Timo Schwarzer"
__email__ = "timo.schwarzer@festo.com"
__status__ = "Experimental"

# ros imports
import rospy

# festo imports
from festo_phand_msgs.msg import SimpleFluidPressures

pressure_pub = rospy.Publisher("/festo/phand/set_pressures", SimpleFluidPressures, queue_size=1)
msg = SimpleFluidPressures()

def close_claw(speed):

    pass

def open_claw(speed):

    pass

def shake_wrist(speed):

    pass

if __name__ == "__main__":
    
    rospy.init_node("test_setpoints")
    rate = rospy.Rate(1)
    
    msg.values = [100000.0] * 12

    step = 0
    while not rospy.is_shutdown():
    
        if step == 0 :
            msg.values[0] = 0.0
            msg.values[1] = 1.0

        if step == 1:
            msg.values[0] = 0.0
            msg.values[1] = 1.0

        if step == 2:
            msg.values[0] = 1.0
            msg.values[1] = 0.0

        print(f"step {step} values: {msg.values}")

        # Iterate to next step
        step +=1
        if step > 2:
            step = 0
        
        # Publish the pressure values
        pressure_pub.publish(msg)
        rate.sleep()
