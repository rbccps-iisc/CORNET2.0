#!/usr/bin/python

"""Setting position of the nodes and enable sockets"""
import subprocess
import sys

from mininet.log import setLogLevel, info
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
import os
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference


def startNAT(root, inetIntf, subnet='10.0/8', localIntf=None):
    """Start NAT/forwarding between Mininet and external network
    root: node to access iptables from
    inetIntf: interface for internet access
    subnet: Mininet subnet (default 10.0/8)"""

    # Identify the interface connecting to the mininet network
    if localIntf == None:
        localIntf = root.defaultIntf()

    # Flush any currently active rules
    root.cmd('iptables -F')
    root.cmd('iptables -t nat -F')

    # Create default entries for unmatched traffic
    root.cmd('iptables -P INPUT ACCEPT')
    root.cmd('iptables -P OUTPUT ACCEPT')
    root.cmd('iptables -P FORWARD DROP')

    # Configure NAT
    root.cmd('iptables -I FORWARD -i', localIntf, '-d', subnet, '-j DROP')
    root.cmd('iptables -A FORWARD -i', localIntf, '-s', subnet, '-j ACCEPT')
    root.cmd('iptables -A FORWARD -i', inetIntf, '-d', subnet, '-j ACCEPT')
    root.cmd('iptables -t nat -A POSTROUTING -o ', inetIntf, '-j MASQUERADE')

    # Instruct the kernel to perform forwarding
    root.cmd('sysctl net.ipv4.ip_forward=1')


def fixNetworkManager(root, intf):
    """Prevent network-manager from messing with our interface,
       by specifying manual configuration in /etc/network/interfaces
       root: a node in the root namespace (for running commands)
       intf: interface name"""
    cfile = '/etc/network/interfaces'
    line = '\niface %s inet manual\n' % intf
    config = open(cfile).read()
    if (line) not in config:
        print('*** Adding', line.strip(), 'to', cfile)
        with open(cfile, 'a') as f:
            f.write(line)
    # Probably need to restart network-manager to be safe -
    # hopefully this won't disconnect you
    root.cmd('service network-manager restart')



def topology(args):
    net = Mininet_wifi(link=wmediumd, wmediumd_mode=interference, noise_th=-91, fading_cof=3)
    #net = Mininet_wifi()
    c1 = net.addController('c1')
    info("*** Creating nodes\n")
    number_of_stations = 2
    staList = []

    net.addStation('sta1', mac='00:00:00:00:00:02', ip='10.0.0.1/8',
                    position='2,10,0')

    net.addStation('sta2', mac='00:00:00:00:00:05', ip='10.0.0.3/8',
                    position='10,10,0')
    info("*** Creating nodes\n")
    position = 60
    #for n in range(number_of_stations):
    #    staList.append(net.addStation(
    #        'sta%s' % (n+1), ip='10.0.0.%s/24' % (n+1), position='70,60,0' ))

    h1 = net.addHost('h1', ip='10.0.0.25/8')#, position='5,0,0')
    ap1 = net.addAccessPoint('ap1', ssid='new-ssid', mode='a', channel='36', position='1,10,0')
    ap2 = net.addAccessPoint('ap2', ssid='ssid-ap2', position='1,45,0')
    internetIface = 'eno1'
    #root = net.addHost('root', ip='10.0.0.254/8', inNamespace=False)
    net.setPropagationModel(model="logDistance", exp=5.5)
    #net.setPropagationModel(model="logNormalShadowing", sL=2, exp=4, variance=2)
    info("*** Configuring wifi nodes\n")
    net.configureWifiNodes()

    info("*** Creating links\n")
    net.addLink(ap1, h1)
    #net.addLink(root, ap1)

    #if '-p' not in args:
    #    net.plotGraph(max_x=500, max_y=500)
    nodes = net.stations + net.aps
    net.telemetry(nodes=nodes, single=True, max_x=100, max_y=100, data_type='position')
    #stat = Stats(nodes)
    #stat.thread_ = thread(target=stat.start)
    #stat.thread_.daemon = True
    #stat.thread_._keep_alive = True
    #stat.thread_.start()



    info("*** Starting Network\n")
    #net.addNAT(linkTo='ap1').configDefault()
    net.build()
    c1.start()
    ap1.start([c1])

    ap2.start([c1])

    #fixNetworkManager( root, 'root-eth0' )

    #startNAT(root, internetIface)

    info("*** Staring Socket Server\n")

    net.socketServer(ip='127.0.0.1', port=12345)


    info("*** Running CLI\n")
    CLI(net)

    info("*** Stopping network\n")
    net.stop()


if __name__ == '__main__':
    os.system('sudo systemctl stop network-manager')
    setLogLevel('debug')
    topology(sys.argv)
