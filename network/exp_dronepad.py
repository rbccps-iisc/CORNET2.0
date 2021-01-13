#!/usr/bin/python

'Setting position of the nodes'

import sys

from mininet.log import setLogLevel, info
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mininet.link import Intf
from mininet.node import Controller, RemoteController, OVSController
from mininet.node import OVSKernelSwitch, UserSwitch


def topology(args):

    net = Mininet_wifi()

    info("*** Creating nodes\n")
    ap1 = net.addAccessPoint('ap1', ssid='new-ssid', mode='g', channel='1',
                             failMode="standalone", mac='00:00:00:00:00:01',
                             position='400,600,0')
    sta1 = net.addStation('sta1', mac='00:00:00:00:00:02', ip='10.0.0.1/8',
                   position='400,400,0')
    #net.addStation('sta2', mac='00:00:00:00:00:03', ip='10.0.0.3/8',
    #               position='70,30,0')
    h1 = net.addHost('h1', ip='10.0.0.2/8')
    #h1 = net.addHost('h1', ip='0.0.0.0')
    #c1 = net.addController('c1')
    c1=net.addController(name='c1',
                      controller=Controller,
                      protocol='tcp',
                      port=6633)

    #info("*** Configuring propagation model\n")
    #net.setPropagationModel(model="logDistance", exp=2)

    # info( '*** Add switches\n')
    # s1 = net.addSwitch('s1')
    # Intf('eno1', node=s1)

    info("*** Configuring wifi nodes\n")
    net.configureWifiNodes()

    info("*** Creating links\n")
    net.addLink(ap1, h1)
    #net.addLink(s1, sta1)


    if '-p' not in args:
        net.plotGraph(max_x=1000, max_y=1000)

    info("*** Starting network\n")
    net.build()
    c1.start()
    #s1.start([c1])
    ap1.start([c1])

    info("*** Running CLI\n")
    CLI(net)

    info("*** Stopping network\n")
    net.stop()


if __name__ == '__main__':
    setLogLevel('info')
    topology(sys.argv)
