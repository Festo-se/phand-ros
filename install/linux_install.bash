#!/usr/bin/env bash

ros_version=`rosversion -d`
if [ $ros_version == "" ]; then
    echo "Please install ROS meldoic on your computer to use the phand_ros tools."
fi

echo "Installing the BionicSoftHand 2.0 python libraries and the ROS workspace"
echo "Creating the directories under /home/" + $USER

# Create the directories
mkdir -p /home/$USER/phand/phand_ws/src
mkdir -p /home/$USER/phand/python_libs

# Change directory
cd /home/$USER/phand/python_libs

# Clone the relevant python library repositories
git clone https://github.com/Schwimo/bionic-message-tools
git clone https://github.com/Schwimo/bionic-pid-control
git clone https://github.com/Schwimo/bionic-dhcp
git clone https://github.com/Schwimo/phand-python-libs

# install the python libraries
pip3 install --editable /home/$USER/phand/python_libs/bionic-dhcp
pip3 install --editable /home/$USER/phand/python_libs/bionic-pid-control
pip3 install --editable /home/$USER/phand/python_libs/bionic-message-tools
pip3 install --editable /home/$USER/phand/python_libs/phand-python-libs

# Change directory and clone the ROS repository
cd /home/$USER/phand/phand_ws/src
git clone https://github.com/Schwimo/phand-ros
rosdep update

sudo apt-get install python3-pip python3-yaml python-catkin-tools python3-dev python3-numpy
sudo pip3 install rospkg catkin_pkg

# build the workspace   
cd /home/$USER/phand/phand_ws/
catkin build

# source the workspace
source /home/$USER/phand/phand_ws/devel/setup.bash
