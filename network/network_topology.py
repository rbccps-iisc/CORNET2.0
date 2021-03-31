#!/usr/bin/python

"""Setting position of the nodes and enable sockets"""

import sys
from mininet.log import setLogLevel, info
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mininet.link import Intf
from mininet.node import Controller, RemoteController, OVSController
from mininet.node import OVSKernelSwitch, UserSwitch