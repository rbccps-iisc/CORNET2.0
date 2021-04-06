#!/usr/bin/python

"""Setting position of the nodes and enable sockets"""

import sys
from mininet.log import setLogLevel, info
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mininet.link import Intf
from mininet.node import Node, Controller, RemoteController, OVSController
from mininet.node import OVSKernelSwitch, UserSwitch
import yaml


def NetworkTopology(config_file):
    "Create a network."
    net = Mininet_wifi()
    with open(config_file) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    models = config['gazebo_models']
    node_type = config['type']
    pose = config['pose']
    ip_list = config['ip_list']

    c1 = net.addController(name='c1',
                           controller=Controller,
                           protocol='tcp',
                           port=6633)

    for idx, node in enumerate(models):
        # print self.type[idx], self.pose[idx]['position']['x'], node
        position = str(pose[idx]['position']['x']) + "," + str(pose[idx]['position']['y']) + "," + str(
            pose[idx]['position']['z'])
        print position

        if node_type[idx] == "STATIC":
            net.addHost(node, ip=ip_list[idx])

            ap1 = net.addAccessPoint('ap1', ssid='new-ssid', mode='g', position=position)
            info("*** Creating links\n")
            net.addLink(ap1, node)
        elif node_type[idx] == "MOBILE":
            net.addStation(node, ip=ip_list[idx], position=position)

    net.setPropagationModel(model="logDistance", exp=3)
    info("*** Configuring wifi nodes\n")
    net.configureWifiNodes()

    #nodes = net.stations
    #net.telemetry(nodes=nodes, single=True, data_type='rssi')
    #net.telemetry(nodes=nodes, single=True, data_type='position')



    #if '-p' not in args:
    net.plotGraph(max_x=10, max_y=10)#, max_z=1000)

    info("*** Starting Network\n")
    #net.addNAT(linkTo='ap1').configDefault()
    net.build()
    c1.start()
    ap1.start([c1])

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
        NetworkTopology(args[1])


if __name__ == '__main__':
    setLogLevel('info')
    sys.exit(main(sys.argv))
