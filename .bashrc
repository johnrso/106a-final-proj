# Filename: .bashrc
# Description: Sources in on the class MASTER version for settings information
# 
# Please (DO NOT) edit this file unless you are sure of what you are doing.
# This file and other dotfiles have been written to work with each other.
# Any change that you are not sure off can break things in an unpredicatable
# ways.

# Set the Class MASTER variable and source the class master version of .cshrc

[[ -z ${MASTER} ]] && export MASTER=${LOGNAME%-*}
[[ -z ${MASTERDIR} ]] && export MASTERDIR=$(eval echo ~${MASTER})

# Set up class wide settings
for file in ${MASTERDIR}/adm/bashrc.d/* ; do [[ -x ${file} ]] && . "${file}"; done

# Set up local settings
for file in ${HOME}/bashrc.d/* ; do [[ -x ${file} ]] && . "${file}"; done

####################
# Sourcing the Robots #
#######################
# If you are working with the Sawyer packages, uncomment the line below
# to source the .bashrc file that sets up ROS with the Sawyer packages:
source /opt/ros/eecsbot_ws/devel/setup.bash

# Otherwise, uncomment the lines below to source the .bashrc file
# that sets up ROS without the Sawyer packages (i.e. when working with T3’s):
#source /opt/ros/noetic/setup.bash
#export TURTLEBOT3_MODEL=burger
##############################
# Configuring the IP Address #
##############################
# Run the following command on a terminal:
# cat /etc/hosts 
# to see the list of IP addresses for the workstations and robots.
# If you are working on a workstation in the lab, your ROS hostname will be your PC IP address.
# Uncomment the line below to set this address, making sure to change the numbers right after
# "192.168.1." to your current workstation number (in this example the workstation number is 8):
export ROS_HOSTNAME=192.168.1.10
# Uncomment the lines below with the correct IP address for the robot that you will be using.
# If you are working with Turtlebots, the ROS Master URI will also be your current PC IP address:
# export ROS_MASTER_URI=http://192.168.1.1:11311
# If working with Sawyer robots, your Master URI will be the IP address of your robot:
export ROS_MASTER_URI=http://alan.local:11311
