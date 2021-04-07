#!/usr/bin/python

"""Setting position of the nodes and enable sockets"""

import sys
from mininet.log import setLogLevel, info
from mininet.link import TCLink
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mn_wifi.topo import Topo
import yaml
import os

class Topology_MN(Topo):
    def __init__(self, *args, **params):
        #super(Topology_MN, self).__init__(*args, **params)

        Topo.__init__(self, *args, **params)
        #pass

    def build(self, config_file):
        with open(config_file) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        models = config['gazebo_models']
        node_type = config['type']
        pose = config['pose']
        ip_list = config['ip_list']

        for idx, node in enumerate(models):
            # print self.type[idx], self.pose[idx]['position']['x'], node
            position = str(pose[idx]['position']['x']) + "," + str(pose[idx]['position']['y']) + "," + str(
                pose[idx]['position']['z'])
            print position

            if node_type[idx] == "STATIC":
                n = self.addHost(node, ip=ip_list[idx])

                ap1 = self.addAccessPoint('ap1', ssid='new-ssid', mode='g', channel='1',
                                         failMode="standalone", position=position)
                info("*** Creating links\n")
                self.addLink(ap1, n, cls=TCLink)
            elif node_type[idx] == "MOBILE":
                self.addStation(node, ip=ip_list[idx], position=position)



def NetworkTopology(config_file):
    "Create a network."
    net = Mininet_wifi()
    with open(config_file) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    models = config['gazebo_models']
    node_type = config['type']
    pose = config['pose']
    ip_list = config['ip_list']
    for idx, node in enumerate(models):
        # print self.type[idx], self.pose[idx]['position']['x'], node
        position = str(pose[idx]['position']['x']) + "," + str(pose[idx]['position']['y']) + "," + str(
            pose[idx]['position']['z'])
        print position

        if node_type[idx] == "STATIC":
            n = net.addHost(node, ip=str(ip_list[idx]))

            ap1 = net.addAccessPoint('ap1', ssid='new-ssid', mode='g', channel='1',
                                     failMode="standalone", position=position)
            info("*** Creating links\n")
            net.addLink(ap1, n)
        elif node_type[idx] == "MOBILE":
            sta1 = net.addStation(node, ip=str(ip_list[idx]), mac='00:00:00:00:00:02', position=position)

    #net.addStation('sta2', ip='10.0.0.4/8', mac='00:00:00:00:00:04', position='10,10,0')

    net.setPropagationModel(model="logDistance", exp=3)
    info("*** Configuring wifi nodes\n")
    net.configureWifiNodes()

    # nodes = net.stations
    # net.telemetry(nodes=nodes, single=True, data_type='rssi')
    # net.telemetry(nodes=nodes, single=True, data_type='position')

    # if '-p' not in args:
    net.plotGraph(max_x=100, max_y=100)  # , max_z=1000)

    info("*** Starting Network\n")
    net.addNAT(linkTo='ap1').configDefault()
    net.build()
    ap1.start([])

    info("*** Staring Socket Server\n")

    net.socketServer(ip='127.0.0.1', port=12345)

    info("*** Running CLI\n")
    CLI(net)

    info("*** Stopping network\n")
    net.stop()


def main(args):
    if len(args) != 2:
        print("usage: network_coordinator.py <config_file>")
    else:
        #NetworkTopology(args[1])
        #'''
        net = Mininet_wifi(topo=Topology_MN(args[1]))
        net.setPropagationModel(model="logDistance", exp=4.5)
        #info("*** Configuring wifi nodes\n")
        #net.configureWifiNodes()
        if '-p' not in args:
            net.plotGraph(max_x=1000, max_y=1000)

        net.setMobilityModel(time=0, model='RandomDirection',
                             max_x=500, max_y=500, seed=20)

        info("*** Starting Network\n")
        net.addNAT(linkTo='ap1').configDefault()
        net.start()
        aps = net.stations
        info("*** printing all nodes\n")
        print (aps)
        info("*** Staring Socket Server\n")
        
        net.socketServer(ip='127.0.0.1', port=12345)
        
        info("*** Running CLI\n")
        CLI(net)
    
        info("*** Stopping network\n")
        net.stop()
        #'''

if __name__ == '__main__':
    os.system('sudo service network-manager stop')
    setLogLevel('info')
    sys.exit(main(sys.argv))
