#!/usr/bin/python
"""
This is the most simple example to showcase Containernet.

Handover example supported by bgscan (Background scanning) and wmediumd.
ieee 802.11r can be enabled adding the parameters below:

ieee80211r='yes'
mobility_domain='a1b2'

e.g. ap1 = net.addAccessPoint('ap1', ..., ieee80211r='yes',
mobility_domain='a1b2',...)

Consider https://w1.fi/cgit/hostap/plain/wpa_supplicant/wpa_supplicant.conf
for more information about bgscan
"""

from containernet.net import Containernet
from containernet.node import DockerSta, Docker
from containernet.cli import CLI
from containernet.link import TCLink
from mininet.log import info, setLogLevel
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference
from mininet.node import Controller, RemoteController
import sys

import os


def topology(args):
    # net = Containernet(controller=Controller, link=wmediumd, wmediumd_mode=interference, noise_th=-91, fading_cof=3)
    net = Containernet(link=wmediumd, wmediumd_mode=interference, noise_th=-91, fading_cof=3)

    info('*** Adding controller\n')
    c0 = net.addController('c0')  # ,controller=RemoteController, protocol='tcp', ip='127.0.0.1', port=6653)
    info('*** Adding docker containers\n')
    robot0 = net.addStation('robot0', ip='10.0.0.2/24', mac='00:02:00:00:00:10',
                            cls=DockerSta, dimage="cornet:focalfoxyNWH", cpu_shares=20, position='2,10,0')
                            #, bgscan_threshold=-60, s_inverval=5, l_interval=10, bgscan_module="simple")
    robot1 = net.addStation('robot1', ip='10.0.0.3/24', mac='00:02:00:00:00:11',
                            cls=DockerSta, dimage="cornet:focalfoxyNWH", cpu_shares=20, position='10,10,0')
                            #, bgscan_threshold=-60, s_inverval=5, l_interval=10, bgscan_module="simple")
    r1 = net.addStation('r1', ip='10.0.0.5/24', cls=DockerSta, dimage="cornet:focalfoxyNWH", cpu_shares=20,
                        position='2,10,0')
                        #,bgscan_threshold=-60, s_inverval=5, l_interval=10, bgscan_module="simple")
    # r2 = net.addStation('r2', ip='10.0.0.6',position='2,10,0')
    ap1 = net.addAccessPoint('ap1', ssid='new-ssid', position='1,10,0')#, ieee80211r='yes')
    # ap1 = net.addStation('ap1', mac='02:00:00:00:01:00', cls=DockerSta, dimage="cornet:focalfoxyNWH",
    #                     ip='10.0.0.1/24', position='1,10,0')
    ap2 = net.addAccessPoint('ap2', ssid='new-ssid', position='1,45,0')#, ieee80211r='yes')
    h1 = net.addHost('h1', ip='10.0.0.4/24', cls=Docker, dimage="cornet:focalfoxyNWH", cpu_shares=20)
    # h2 = net.addHost('h2', ip='10.0.0.5/8', cls=Docker, dimage="cornet:focalfoxyNWH")

    info("*** Configuring Propagation Model\n")
    net.setPropagationModel(model="logDistance", exp=5.5)

    info('*** Configuring WiFi nodes\n')
    net.configureWifiNodes()

    s1 = net.addSwitch('s1')
    # s2 = net.addSwitch('s2')
    # s3 = net.addSwitch('s3')

    info("*** Creating links\n")
    net.addLink(s1, h1, cls=TCLink)
    net.addLink(s1, ap1, cls=TCLink)
    net.addLink(s1, ap2, cls=TCLink)

    if '-p' not in args:
        net.plotGraph(max_x=100, max_y=100)

    info('*** Starting network\n')
    net.build()
    # ap1.cmd('echo 1 > /proc/sys/net/ipv4/ip_forward')
    # ap1.setIP('10.0.0.1/24', intf='ap1-wlan0')
    # ap1.setIP('10.0.1.10/24', intf='ap1-eth2')

    # robot1.cmd('route add -net 10.0.0.0/24 gw 10.0.0.1')
    # robot0.cmd('route add -net 10.0.0.0/24 gw 10.0.0.1')
    # h1.cmd('route add -net 10.0.0.0/24 gw 10.0.0.1')
    c0.start()
    s1.start([c0])
    ap1.start([c0])
    ap2.start([c0])

    # ap2.start([c0])

    robot1.cmd('route add default gw 10.0.0.1 robot1-wlan0')
    robot0.cmd('route add default gw 10.0.0.1 robot0-wlan0')
    r1.cmd('route add default gw 10.0.0.1 r1-wlan0')
    h1.cmd('route add default gw 10.0.0.1 h1-eth0')
    s1.cmd("ovs-ofctl add-flow s1 priority=1,arp,actions=flood")
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
    setLogLevel('debug')
    topology(sys.argv)
