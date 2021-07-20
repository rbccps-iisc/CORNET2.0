#!/usr/bin/python
"""
This is the most simple example to showcase Containernet.
"""
from containernet.net import Containernet
from containernet.node import DockerSta
from containernet.cli import CLI
from containernet.term import makeTerm
from mininet.log import info, setLogLevel
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference

import sys

import os

def topology():
    net = Containernet(link=wmediumd, wmediumd_mode=interference, noise_th=-91, fading_cof=3)

    info('*** Adding docker containers\n')
    sta1 = net.addStation('sta1', ip='10.0.0.3', mac='00:02:00:00:00:10',
                          cls=DockerSta, dimage="cornet:focalfoxyNWH", cpu_shares=20)
    sta2 = net.addStation('sta2', ip='10.0.0.4', mac='00:02:00:00:00:11',
                          cls=DockerSta, dimage="cornet:focalfoxyNWH", cpu_shares=20)
    ap1 = net.addAccessPoint('ap1')
    c0 = net.addController('c0')

    info('*** Configuring WiFi nodes\n')
    net.configureWifiNodes()

    info('*** Starting network\n')
    net.build()
    ap1.start([c0])

    makeTerm(sta1, cmd="bash -c 'apt-get update && apt-get install iw;'")
    makeTerm(sta2, cmd="bash -c 'apt-get update && apt-get install iw;'")

    #sta1.cmd('iw dev sta1-wlan0 connect new-ssid')
    #sta2.cmd('iw dev sta2-wlan0 connect new-ssid')

    info('*** Running CLI\n')
    CLI(net)

    info('*** Stopping network\n')
    net.stop()


if __name__ == '__main__':
    os.system('sudo service network-manager stop')
    setLogLevel('debug')
    topology()

