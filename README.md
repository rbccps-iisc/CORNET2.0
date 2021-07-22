# CORNET2.0
Co-Simulation Framework for applications involving Networked Robots like Connected Autonomous Vehicles, Network of UAVs. 

---
 
CORNET 1.0 integrates Ardupilot SITL in Gazebo and NS3 and the source code is available [here](https://github.com/srikrishna3118/CORNET.git).

CORNET 2.0 is more generic framework that can be integrated with any robotic framework that supports ROS. This code was tested on an Ubuntu 16.04 and Ubuntu 18.04 system.

For network realization, we use Mininet Wi-Fi; the dependent packages need to installed as mentioned in mininet wifi documentation. 

##Installation 
###mininet-WiFi + Containernet Support

**We highly recommend using Ubuntu version 16.04 or higher. Some new hostapd features might not work on Ubuntu 14.04.**

     sudo apt-get install git
     sudo apt-get install ansible git aptitude
     git clone https://github.com/ramonfontes/containernet.git
     cd containernet/ansible
     ~/containernet/ansible$ sudo ansible-playbook -i "localhost," -c local install.yml
     cd ..
     ~/containernet$ sudo python setup.py install



###ROS2
The Docker with ROS2 support provided in docker_container folder along with other wireless tools for testing and evaluation.

build.sh script will build and tag the images required for running. you can add you own Dockerfile based on the file Dockerfile
