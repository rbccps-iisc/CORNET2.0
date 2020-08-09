#!/usr/bin/python

'Setting position of the nodes and enable sockets'

from mininet.log import setLogLevel, info
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi

def topology():

    net = Mininet_wifi()

    info("*** Creating nodes\n")
    net.addStation('sta1', mac='00:00:00:00:00:02', ip='10.0.0.1/8',
                   position='30,60,0')
    ap1 = net.addAccessPoint('ap1',ssid='new-ssid',mode='g',position='50,50,0')
    net.setPropagationModel(model="logDistance", exp=4.5)
    info("*** Configuring wifi nodes\n")
    net.configureWifiNodes()

    info("*** Creating Links\n")


    info("*** Starting Network\n")
    net.addNAT(linkTo='ap1').configDefault()
    net.build()
    ap1.start([])

    info("*** Staring Socket Server\n")

    net.socketServer(ip='127.0.0.1',port=12345)

    info("*** Running CLI\n")
    CLI(net)

    info("*** Stopping network\n")
    net.stop()


if __name__== '__main__':
    setLogLevel('info')
    topology()
