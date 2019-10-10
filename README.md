# PHAND ROS PROJECT
[![FESTO](logo.png)](https://www.festo.com/group/de/cms/10156.htm)
The pHand ROS project provides the implementation to control the BionicSoftHand with ROS.

| IMPORTANT: The maximum supply pressure for the BionicSoftHand is 3 bars. If you connect more than this amount of air pressure the hand gets damaged and is not usable anymore. |
| --- |

| Warning: Up to 300V is used for the piezo valves. Be careful with the back part of the valve terminal |
| --- |


## PROLOG
The BionicSoftHand is a pneumatic gripper with 12 independent controllable chambers.

## REQUIREMENTS
The ROS package uses the bionic_python_libs which provide the core functionality to communicate with the BionicSoftHand. 
These packages have to be installed on your system. 
This is done with executing the script `install_linux.bash` inside the bionic_python_libs repository 
which installs the libraries. 

The bionic_python_libs libraries require python3. The default python for ROS up to melodic is python2. 
To make ROS work with python3 you have to install the following packages: 
```
sudo apt-get install python3-pip python3-yaml python-catkin-tools python3-dev python3-numpy
sudo pip3 install rospkg catkin_pkg
```


## Launchfiles
To start the hardware interface execute the launch file.
```
roslaunch festo_phand_driver hardware_interface.launch
```

## ROS topics
**The hand publishes the following topics:**
```
festo/phand/state
```
With the current state of the hand and internal sensors

Each connected "external" sensor is published on its own topic:
```
festo/phand/connected_sensors/%sensor_name%
```

**The hand subscribes to the following topic:**
```
festo/phand/set_valve_setpoints
```
To set the valves publish the corresponding `ValveSetPoints` message to the topic.
The values should be between 0.0 and 1.0. 0 Meaning 0V for the piezo valve and 1.0 meaning 300V for the piezo valve.

## ROS services
The following services are offered by the hardware interface:

```
/festo/phand/close
```
To close the hand 
```
/festo/phand/open
```
To open the hand
```
/festo/phand/set_configuration
```
To change the configuration of the hand

| WARNING: not yet implemented ! |
| --- |