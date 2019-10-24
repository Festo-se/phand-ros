
ValveSetPoints valves # current values

# Contains 14 pressure values. All 12 chambers and exhaust and supply pressure.
SimpleFluidPressures actual_pressures       # Current fluid pressures in measured in the valve terminal in Pascal
SimpleFluidPressures set_pressures          # Current set fluid pressures
sensor_msgs/Imu imu                 # Imu data from sensor of the hand
sensor_msgs/MagneticField mag       # Imu data from sensor of the hand