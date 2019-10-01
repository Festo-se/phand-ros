# Possible control modes
byte NO_CTRL=0              # All valves closed sensors being read. Default mode after boot
byte VALVE_CTRL=1           # Valves can be controlled directly
byte PRESSURE_CTRL=2        # Pressures can be controlled directly
byte POSITION_CTRL=3        # Position control
byte GRIPPER_MODE=4         # Simplyfied control mode, for using the hand a simple gripper

byte mode
