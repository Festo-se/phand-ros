# Possible hand states
byte OFFLINE=1              # Hand not connected
byte ONLINE=2              # Hand connected and ready
byte ERROR=3               # Hand connected and in error state


Header header
string hand_id              # unique id identifying the hand

byte state                  # Hand state as described above
ControlMode mode            # Current control mode as described above
diagnostic_msgs/KeyValue[] status_codes         # an array of values associated with the status

string[] connected_sensor_names
int16[] connected_sensor_ids

HandSensors internal_sensors
