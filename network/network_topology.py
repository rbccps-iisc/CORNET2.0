#!/usr/bin/python

"""Setting position of the nodes and enable sockets"""

import sys
from mininet.log import setLogLevel, info
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mn_wifi.topo import MN_TOPO
from mininet.link import Intf
from mininet.node import Controller, RemoteController, OVSController
from mininet.node import OVSKernelSwitch, UserSwitch
import yaml


class NetworkTopology(MN_TOPO):
    def __init__(self, config_file):

        with open(config_file) as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

        self.models = self.config['gazebo_models']
        self.type = self.config['type']
        self.pose = self.config['pose']
        self.ip_list = self.config['ip_list']

    def build(self):
        info("*** Creating nodes\n")
        for idx, node in enumerate(self.models):
            # print self.type[idx], self.pose[idx]['position']['x'], node
            position = str(self.pose[idx]['position']['x']) + "," + str(self.pose[idx]['position']['y']) + "," + str(
                self.pose[idx]['position']['z'])
            print position

            if self.type[idx] == "STATIC":
                self.addHost(node, ip=self.ip_list[idx])

                ap1 = self.addAccessPoint('ap1', ssid='new-ssid', position=position)
                self.addLink(ap1, node)
            elif self.type[idx] == "MOBILE":
                self.addStation(node, ip=self.ip_list[idx], position=position)


def main(args):
    if len(args) != 2:
        print("usage: network_coordinator.py <config_file>")
    else:
        net = Mininet_wifi(topo=NetworkTopology(args[1]))
        net.setPropagationModel(model="logDistance", exp=3)
        info("*** Configuring wifi nodes\n")
        net.configureWifiNodes()
        c1 = net.addController(name='c1',
                               controller=Controller,
                               protocol='tcp',
                               port=6633)
        if '-p' not in args:
            net.plotGraph(max_x=1000, max_y=1000, max_z=1000)

        info("*** Starting Network\n")
        net.addNAT(linkTo='ap1').configDefault()
        net.build()
        c1.start()
        ap1.start([c1])

        info("*** Staring Socket Server\n")

        net.socketServer(ip='127.0.0.1', port=12345)

        info("*** Running CLI\n")
        CLI(net)

        info("*** Stopping network\n")
        net.stop()


if __name__ == '__main__':
    sys.exit(main(sys.argv))
