#!/usr/bin/env python
import rospy
from festo_phand_msgs.msg import GenericSensor


class Loomia2UltrahapticsBridge:

    def __init__(self):

        rospy.Subscriber("/festo/phand/connected_sensors/loomia_sensors", GenericSensor, self.new_sensor_data_cb)
        self.pub = rospy.Publisher("/festo/phand/loomia_processed", GenericSensor, queue_size=1)

        self.processed_data = GenericSensor()
        self.processed_data.calibrated_values = [0]*6

        self.processed_data.name = "Loomia sensors mapped to: PALM:THUMB:INDEX:MIDDLE:RING:PINKY"

        self.indexes = [[39, 40, 41, 42, ]] #Palm
        self.indexes.append([106, 108]) #Thumb
        self.indexes.append([12, 23, 26]) #Index
        self.indexes.append([55, 37, 59]) #Midle
        self.indexes.append([80, 77, 69]) #Ring
        self.indexes.append([91, 102]) #Pinky

        self.threshold = 1000

    def spin(self):
        rospy.spin()

    def new_sensor_data_cb(self, data):


        for sensor, idxs in enumerate(self.indexes):

            count = 0
            for idx in idxs:

                if data.raw_values[idx] > self.threshold:
                    count += 1

            if count > 0:
                self.processed_data.calibrated_values[sensor] = 1
            else:
                self.processed_data.calibrated_values[sensor] = 0

        print(self.processed_data.calibrated_values)
        self.pub.publish(self.processed_data)




if __name__ == '__main__':
    rospy.init_node("loomia_ultra_haptics_bridge")
    bridge = Loomia2UltrahapticsBridge()
    bridge.spin()