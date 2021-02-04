#! /usr/bin/env python3


class phandHelpers:

    def __init__(self):
        # Hand desired pressure values
        self.wrist_p = np.zeros(2)
        self.thumb_p = np.zeros(3)
        self.index_p = np.zeros(3)
        self.middle_p = np.zeros(1)
        self.ring_p = np.zeros(1)
        self.pinky_p = np.zeros(1)

        # Publisher and subscriber
        self.pressure_message = SimpleFluidPressures()
        self.pressure_publisher = rospy.Publisher("/festo/phand/set_pressures", SimpleFluidPressures, queue_size=10) 


    def move_thumb(self, upper, lower, side):
        # update new pressure values
        self.thumb_p = [upper, lower, side]

        # send new values to hand
        pressure_message = SimpleFluidPressures()
        pressure_message.values = [self.thumb_p[2], self.thumb_p[1], 400000, self.thumb_p[0], self.index_p[0], self.wrist_p[0], self.index_p[1], self.wrist_p[1], self.middle_p[0], self.index_p[2], self.ring_p[0], self.pinky_p[0]] 
        self.pressure_publisher.publish(pressure_message)
        return


    def move_index(self, upper, lower, str):
        # update new pressure values
        self.index_p = [upper, lower, side]

        # send new values to hand
        pressure_message = SimpleFluidPressures()
        pressure_message.values = [self.thumb_p[2], self.thumb_p[1], 400000, self.thumb_p[0], self.index_p[0], self.wrist_p[0], self.index_p[1], self.wrist_p[1], self.middle_p[0], self.index_p[2], self.ring_p[0], self.pinky_p[0]] 
        self.pressure_publisher.publish(pressure_message)
        return


    def move_mrp(self, pressure):
        # update new pressure values
        self.middle_p = [pressure]
        self.ring_p = [pressure]
        self.pinky_p = [pressure]

        # send new values to hand
        pressure_message = SimpleFluidPressures()
        pressure_message.values = [self.thumb_p[2], self.thumb_p[1], 400000, self.thumb_p[0], self.index_p[0], self.wrist_p[0], self.index_p[1], self.wrist_p[1], self.middle_p[0], self.index_p[2], self.ring_p[0], self.pinky_p[0]] 
        self.pressure_publisher.publish(pressure_message)
        

    def set_thumb_side(self):
        self.thumb_p[2] = 460000
        pressure_message = SimpleFluidPressures()
        pressure_message.values = [self.thumb_p[2], self.thumb_p[1], 400000, self.thumb_p[0], self.index_p[0], self.wrist_p[0], self.index_p[1], self.wrist_p[1], self.middle_p[0], self.index_p[2], self.ring_p[0], self.pinky_p[0]] 
        self.pressure_publisher.publish(pressure_message)
        

    def set_thumb_close(self):
        self.thumb_p[2] = 410000
        self.thumb_p[1] = 370000
        self.thumb_p[0] = 360000
        pressure_message = SimpleFluidPressures()
        pressure_message.values = [self.thumb_p[2], self.thumb_p[1], 400000, self.thumb_p[0], self.index_p[0], self.wrist_p[0], self.index_p[1], self.wrist_p[1], self.middle_p[0], self.index_p[2], self.ring_p[0], self.pinky_p[0]] 
        self.pressure_publisher.publish(pressure_message)

    def set_thumb_open(self):
        self.thumb_p[2] = 570000
        self.thumb_p[1] = 150000
        self.thumb_p[0] = 150000
        pressure_message = SimpleFluidPressures()
        pressure_message.values = [self.thumb_p[2], self.thumb_p[1], 400000, self.thumb_p[0], self.index_p[0], self.wrist_p[0], self.index_p[1], self.wrist_p[1], self.middle_p[0], self.index_p[2], self.ring_p[0], self.pinky_p[0]] 
        self.pressure_publisher.publish(pressure_message)
        
    def set_index_side(self):
        self.index_p[2] = 570000
        pressure_message = SimpleFluidPressures()
        pressure_message.values = [self.thumb_p[2], self.thumb_p[1], 400000, self.thumb_p[0], self.index_p[0], self.wrist_p[0], self.index_p[1], self.wrist_p[1], self.middle_p[0], self.index_p[2], self.ring_p[0], self.pinky_p[0]] 
        self.pressure_publisher.publish(pressure_message)


    def set_index_close(self):
        self.index_p[0] = 570000
        self.index_p[1] = 570000
        pressure_message = SimpleFluidPressures()
        pressure_message.values = [self.thumb_p[2], self.thumb_p[1], 400000, self.thumb_p[0], self.index_p[0], self.wrist_p[0], self.index_p[1], self.wrist_p[1], self.middle_p[0], self.index_p[2], self.ring_p[0], self.pinky_p[0]] 
        self.pressure_publisher.publish(pressure_message)
        

    def set_mrp_close(self):
        self.middle_p[0] = 570000
        self.ring_p[0] = 560000
        self.pinky_p[0] = 560000

        pressure_message = SimpleFluidPressures()
        pressure_message.values = [self.thumb_p[2], self.thumb_p[1], 400000, self.thumb_p[0], self.index_p[0], self.wrist_p[0], self.index_p[1], self.wrist_p[1], self.middle_p[0], self.index_p[2], self.ring_p[0], self.pinky_p[0]] 
        self.pressure_publisher.publish(pressure_message)
        

    def set_hand_open(self):    
        # set finger pressure values
        self.thumb_p[2] =  100000
        self.thumb_p[1] =  100000
        self.thumb_p[0] = 100000
        self.index_p[0] = 100000
        self.index_p[1] = 100000
        self.middle_p[0] = 100000
        self.index_p[2] = 100000
        self.ring_p[0] = 100000
        self.pinky_p[0] = 100000

        # send new values to hand
        pressure_message = SimpleFluidPressures()
        pressure_message.values = [self.thumb_p[2], self.thumb_p[1], 400000, self.thumb_p[0], self.index_p[0], self.wrist_p[0], self.index_p[1], self.wrist_p[1], self.middle_p[0], self.index_p[2], self.ring_p[0], self.pinky_p[0]] 
        self.pressure_publisher.publish(pressure_message)
        return


    def set_hand_fist(self):
        # set finger pressure values
        self.thumb_p[2] =  320000
        self.thumb_p[1] =  370000
        self.thumb_p[0] = 360000
        self.index_p[0] = 570000
        self.index_p[1] = 570000
        self.middle_p[0] = 570000
        self.index_p[2] = 590000
        self.ring_p[0] = 560000
        self.pinky_p[0] = 560000

        # send new values to hand
        pressure_message = SimpleFluidPressures()
        pressure_message.values = [self.thumb_p[2], self.thumb_p[1], 400000, self.thumb_p[0], self.index_p[0], self.wrist_p[0], self.index_p[1], self.wrist_p[1], self.middle_p[0], self.index_p[2], self.ring_p[0], self.pinky_p[0]] 
        self.pressure_publisher.publish(pressure_message)
        return
