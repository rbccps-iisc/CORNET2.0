#!/usr/bin/python

"""Setting position of the nodes and enable sockets"""
import sys
import time
from mininet.log import setLogLevel, info
from mininet.link import TCLink
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mn_wifi.topo import Topo
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference
import yaml
import os
start = time.time()

class Topology_MN(Topo):
    def __init__(self, *args, **params):
        super(Topology_MN, self).__init__(*args, **params)
        #Topo.__init__(self, *args, **params)

    def build(self, config_file):
        with open(config_file) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        models = config['gazebo_models']
        node_type = config['type']
        pose = config['pose']
        ip_list = config['ip_list']

        #hosts = []
        aps = []
        robots = []

        for idx, node in enumerate(models):
            # print self.type[idx], self.pose[idx]['position']['x'], node
            position = str(pose[idx]['position']['x']) + "," + str(pose[idx]['position']['y']) + "," + str(
                pose[idx]['position']['z'])
            print position
            print idx, node
            if node_type[idx] == "STATIC":
                robots.append(self.addHost(node, ip=ip_list[idx]))#, position=position)


                aps.append(self.addAccessPoint('ap1', ssid='new-ssid', mode='g', channel='1',
                                         failMode="standalone", position=position))
                info("*** Creating links\n")
                self.addLink(robots[idx], aps[idx], cls=TCLink)
            elif node_type[idx] == "MOBILE":
                robots.append(self.addStation(node, ip=ip_list[idx], position=position))
                aps.append(0)

        print robots, aps


def main(args):
    if len(args) != 2:
        print("usage: network_coordinator.py <config_file>")
    else:

        net = Mininet_wifi(topo=Topology_MN(args[1]),link=wmediumd,
                       wmediumd_mode=interference)
        net.setPropagationModel(model="logDistance", exp=4)
        info("*** Configuring wifi nodes\n")
        #net.configureWifiNodes()
        if '-p' not in args:
            net.plotGraph(max_x=1000, max_y=1000)
        nodes = net.stations  # + net.aps
        #net.telemetry(nodes=nodes, single=True, data_type='rssi')

        info("*** Starting Network\n")
        #net.addNAT(linkTo='ap1').configDefault()
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


if __name__ == '__main__':
    os.system('sudo service network-manager stop')
    setLogLevel('info')
    sys.exit(main(sys.argv))
