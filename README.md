# Manual control of a Kuka LWR arm with Phantom Omni haptic controller device with firewire interface

1. Before cloning this project, follow the instructions in the following links to instal the Linux and ROS drivers for the Phantom omni

https://fsuarez6.github.io/projects/geomagic-touch-in-ros/

https://github.com/fsuarez6/phantom_omni/releases
 
https://github.com/jaejunlee0538/sensable_phantom_in_linux


2. Test the installation with 

roslaunch omni_common omni.launch

If you faced the following error when running the any of the omni launch files in the ROS driver packages,

[ERROR] [1526311017.740632652]: Failed to initialize haptic device

In the terminal run

sudo chmod 666 /dev/fw*

to give permissions to the operating system to open the device ports on the FireWire card.

3. Now clone the iros18_manual repository into your ROS workspace and catkin_make the package. 

4. Eventually, you can run the code used for IROS 2018 submission with the following command

roslaunch iros18_manual manual.launch
