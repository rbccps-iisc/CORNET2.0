#!/usr/bin/python

from mininet.node import Controller, OVSKernelSwitch, Host
from mininet.log import setLogLevel, info
from mn_wifi.net import Mininet_wifi
from mn_wifi.node import Station, OVSKernelAP
from mn_wifi.cli import CLI
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference
from subprocess import call
import os

def myNetwork():

    net = Mininet_wifi(topo=None,
                       build=False,
                       link=wmediumd,
                       wmediumd_mode=interference,
                       ipBase='10.0.0.0/8')

    info( '*** Adding controller\n' )
    c0 = net.addController(name='c0',
                           controller=Controller,
                           protocol='tcp',
                           port=6633)

    info( '*** Add switches/APs\n')
    ap1 = net.addAccessPoint('ap1', cls=OVSKernelAP, ssid='ap1-ssid', model='DI524',
                             channel='1', mode='g', position='600.0,400.0,0')
    s1 = net.addSwitch('s1', cls=OVSKernelSwitch)

    info('*** Add hosts/stations\n')
    sta1 = net.addStation('sta1', ip='10.0.0.1', antennaHeight='1', antennaGain='5')#,
                           #position='400,400,0')
    #sta3 = net.addStation('sta3', ip='10.0.0.3',
                           #position='600.0,200.0,0')
    h1 = net.addHost('h1', cls=Host, ip='10.0.0.2', defaultRoute=None)

    info("*** Configuring Propagation Model\n")
    net.setPropagationModel(model="logDistance", exp=3)

    info("*** Configuring wifi nodes\n")
    net.configureWifiNodes()

    info( '*** Add links\n')
    net.addLink(s1, ap1)
    net.addLink(h1, s1)

    net.plotGraph(max_x=1000, max_y=1000)
    #net.setMobilityModel(time=0, model='RandomWayPoint', max_x=1000, max_y=1000,
    #                     min_v=0.5, max_v=0.5, seed=20)

    net.startMobility(time=0, mob_rep=1, reverse=False)

    p1, p2 = dict(), dict()

    p1 = {'position': '250.0,400.0,0.0'}
    p2 = {'position': '950.0,400.0,0.0'}
    #p2 = {'position': '600.0,400.0,0.0'}

    net.mobility(sta1, 'start', time=1, **p1)
    net.mobility(sta1, 'stop', time=100, **p2)
    net.stopMobility(time=105)


    info( '*** Starting network\n')
    net.build()
    info( '*** Starting controllers\n')
    for controller in net.controllers:
        controller.start()

    info( '*** Starting switches/APs\n')
    net.get('ap1').start([c0])
    net.get('s1').start([c0])

    info( '*** Post configure nodes\n')

    CLI(net)
    net.stop()


if __name__ == '__main__':
    os.system('sudo service network-manager stop')
    setLogLevel( 'info' )
    myNetwork()

