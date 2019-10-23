#!/usr/bin/env python
import rospy
from festo_phand_msgs.msg import ValveSetPoints


rospy.init_node("test_setpoints")

pub = rospy.Publisher("/festo/phand/set_valve_setpoints", ValveSetPoints, queue_size=1)


rate = rospy.Rate(1)

msg = ValveSetPoints()
msg.exhaust_valve_setpoints = [0.0]*12

step = 0
while not rospy.is_shutdown():

    if step == 0 :
        msg.exhaust_valve_setpoints[0] = 0.0
        msg.exhaust_valve_setpoints[1] = 1.0

    if step == 1:
        msg.exhaust_valve_setpoints[0] = 0.0
        msg.exhaust_valve_setpoints[1] = 1.0

    if step == 2:
        msg.exhaust_valve_setpoints[0] = 1.0
        msg.exhaust_valve_setpoints[1] = 0.0

    print("step %i values: %i %i"%(step, msg.exhaust_valve_setpoints[0], msg.exhaust_valve_setpoints[1]))

    step +=1
    if step > 2:
        step = 0
    pub.publish(msg)
    rate.sleep()