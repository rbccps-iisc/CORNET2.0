#!/usr/bin/python

from containernet.net import Containernet
from containernet.node import DockerSta, Docker
from containernet.cli import CLI
from containernet.term import makeTerm
from containernet.link import TCLink
from mininet.log import info, setLogLevel
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference
from subprocess import call
import os

def myNetwork():

    net = Containernet(topo=None,
                       build=False,
                       link=wmediumd,
                       wmediumd_mode=interference,
                       ipBase='10.0.0.0/8')


    info( '*** Add switches/APs\n')
    ap1 = net.addAccessPoint('ap1', ssid='new-ssid1', position='50,50,0', failMode="standalone")

    n = 5;
    sta_list = []
    info('*** Add hosts/stations\n')
    for i in range(0, n):
        sta_list.append(net.addStation('sta%s'%i, ip='10.0.0.%s'%i, antennaHeight='1', antennaGain='5',
                                       cls=DockerSta, dimage="cornet:focalfoxyNWH"))
    #sta1 = net.addStation('sta1', ip='10.0.0.1', antennaHeight='1', antennaGain='5')#,
                           #position='400,400,0')
    #sta3 = net.addStation('sta3', ip='10.0.0.3',
                           #position='600.0,200.0,0')
    #h1 = net.addHost('h1', cls=Host, ip='10.0.0.2', defaultRoute=None)

    info("*** Configuring Propagation Model\n")
    net.setPropagationModel(model="logDistance", exp=3)

    info("*** Configuring wifi nodes\n")
    net.configureWifiNodes()

    info( '*** Add links\n')
    #net.addLink(s1, ap1)
    #net.addLink(h1, s1)

    net.plotGraph(max_x=1000, max_y=1000)
    net.setMobilityModel(time=0, model='RandomWayPoint', max_x=100, max_y=100,
                         min_v=0.5, max_v=0.5, seed=20)

    #net.startMobility(time=0, mob_rep=1, reverse=False)

    p1, p2 = dict(), dict()


    #p1 = {'position': '250.0,400.0,0.0'}
    #p2 = {'position': '950.0,400.0,0.0'}
    #p2 = {'position': '600.0,400.0,0.0'}

    #net.mobility(sta1, 'start', time=1, **p1)
    #net.mobility(sta1, 'stop', time=100, **p2)
    #net.stopMobility(time=105)


    info( '*** Starting network\n')
    net.build()

    info( '*** Starting switches/APs\n')
    ap1.start([])
    ap1.cmd("ovs-ofctl add-flow ap1 priority=1,arp,actions=flood")
    #net.get('s1').start([c0])

    info( '*** Post configure nodes\n')
    for sta in sta_list:
        sta.cmd('nmap -sP 10.0.0.1/24')
    CLI(net)
    os.system('sudo service network-manager start')
    net.stop()


if __name__ == '__main__':
    os.system('sudo service network-manager stop')
    setLogLevel( 'info' )
    myNetwork()

