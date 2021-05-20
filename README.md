# CORNET2.0
Co-Simulation Framework for applications involving Networked Robots like Connected Autonomous Vehicles, Network of UAVs. 

---
 
CORNET 1.0 integrates Ardupilot SITL in Gazebo and NS3 and the source code is available [here](https://github.com/srikrishna3118/CORNET.git).

CORNET 2.0 is more generic framework that can be integrated with any robotic framework that supports ROS. This code was tested on an Ubuntu 16.04 and Ubuntu 18.04 system.

For network realization Mininet WiFi is used; the dependent packages need to installed as mentioned in mininet wifi documentation. 

##Installation 
###mininet-WiFi

**We highly recommend using Ubuntu version 16.04 or higher. Some new hostapd features might not work on Ubuntu 14.04.**

step 1: $ sudo apt-get install git  
step 2: $ git clone https://github.com/intrig-unicamp/mininet-wifi  
step 3: $ cd mininet-wifi  
step 4: $ sudo util/install.sh -Wlnfv  

###protobuf

setp 1: $ wget https://github.com/protocolbuffers/protobuf/releases/download/v3.17.0/protobuf-all-3.17.0.tar.gz
setp 2: $ tar -xzvf protobuf-all-3.17.0.tar.gz
setp 3: $ ./configure
setp 4: $ make check
setp 5: $ sudo ldconfig
setp 6: $ protoc --version

###ROS

TO BE CONTINUED ...
