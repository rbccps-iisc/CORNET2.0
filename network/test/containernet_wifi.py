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


def topology(args):
    image = "cornet:focalfoxyNWH"
    net = Containernet(link=wmediumd, wmediumd_mode=interference, noise_th=-91, fading_cof=3)

    info('*** Adding docker containers\n')
    sta1 = net.addStation('name_robo0', ip='10.0.0.1/8', mac='00:02:00:00:00:10',
                          cls=DockerSta, dimage=image, cpu_shares=20, position='2,10,0')
    sta2 = net.addStation('name_robo1', ip='10.0.0.2/8', mac='00:02:00:00:00:11',
                          cls=DockerSta, dimage=image, cpu_shares=20, position='10,10,0')
    ap1 = net.addAccessPoint('ap_0', ssid='new-ssid', mode='g', channel='6', position='1,1,0')
    #h1 = net.addHost('h1', ip='10.0.0.2/8',
     #                dimage=image)

    c0 = net.addController('c0')

    info("*** Configuring Propagation Model\n")
    net.setPropagationModel(model="logDistance", exp=4)

    info('*** Configuring WiFi nodes\n')
    net.configureWifiNodes()

    #info("*** Creating links\n")
    #net.addLink(ap1, h1)
    if '-p' not in args:
        net.plotGraph(max_x=200, max_y=200)

    info('*** Starting network\n')
    net.build()
    ap1.start([c0])

    # makeTerm(sta1, cmd="bash -c 'apt-get update && apt-get install iw;'")
    # makeTerm(sta2, cmd="bash -c 'apt-get update && apt-get install iw;'")

    # sta1.cmd('iw dev mybot1-wlan0 connect new-ssid')
    # sta2.cmd('iw dev mybot2-wlan0 connect new-ssid')

    net.socketServer(ip='127.0.0.1', port=12345)

    info('*** Running CLI\n')
    CLI(net)
    os.system('sudo service network-manager start')

    info('*** Stopping network\n')
    net.stop()


if __name__ == '__main__':
    os.system('sudo service network-manager stop')
    setLogLevel('info')
    topology(sys.argv)
