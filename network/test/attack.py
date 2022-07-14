#!/usr/bin/python
'Setting the position of nodes and providing mobility'
import os
import sys
from mininet.log import setLogLevel, info
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference

def topology(args):
    "Create a network."
    net = Mininet_wifi(link=wmediumd, wmediumd_mode=interference, noise_th=-91, fading_cof=3)
    c1 = net.addController('c1')
    info("*** Creating nodes\n")
    number_of_stations = 2
    staList = []
    for n in range(number_of_stations):
       staList.append(net.addStation(
           'sta%s' % (n+1), ip='10.0.0.%s/24' % (n+1), position='70,60,0' ))

    h1 = net.addHost('h1', ip='10.0.0.25/8')  # , position='5,0,0')
    ap1 = net.addAccessPoint('ap1', ssid='new-ssid', mode='a', channel='36', position='1,10,0')

    net.setPropagationModel(model="logDistance", exp=2)

    info("*** Configuring wifi nodes\n")
    net.configureWifiNodes()

    info("*** Creating links\n")
    net.addLink(ap1, h1)

    # if '-p' not in args:
    #     net.plotGraph(max_x=100, max_y=100)

    nodes = net.stations
    net.telemetry(nodes=nodes, single=True, data_type='rssi')

    net.build()
    c1.start()
    ap1.start([c1])
    info("*** Running CLI\n")
    CLI(net)
    info("*** Stopping network\n")
    net.stop()

if __name__ == '__main__':
    os.system('sudo systemctl stop network-manager')
    setLogLevel('info')
    topology(sys.argv)
