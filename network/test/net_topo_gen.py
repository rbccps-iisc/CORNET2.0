
from containernet.net import Containernet
from containernet.node import DockerSta,Docker
from containernet.cli import CLI
from containernet.link import TCLink
from containernet.term import makeTerm
from mn_wifi.topo import Topo
from mininet.log import info, setLogLevel
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference
import sys
import os
import yaml


class Topology_MN(Topo):
    def __init__(self, *args, **params):
        super(Topology_MN, self).__init__(*args, **params)
        # Topo.__init__(self, *args, **params)

    def build(self, config_file):
        with open(config_file) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        models = config['gazebo_models']
        node_type = config['type']
        pose = config['pose']
        ip_list = config['ip_list']

        # hosts = []
        #aps = []
        robots = []

        for idx, node in enumerate(models):
            # print self.type[idx], self.pose[idx]['position']['x'], node
            position = str(pose[idx]['position']['x']) + "," + str(pose[idx]['position']['y']) + "," + str(
                pose[idx]['position']['z'])
            print (position)
            print (idx, node)
            if node_type[idx] == "STATIC":
                robots.append(self.addHost(node,cls=Docker, dimage="cornet:focalfoxyNWH", ip=ip_list[idx]))  # , position=position)

                aps = (self.addAccessPoint('ap1', ip='10.0.0.10', ssid='new-ssid', mode='g', position=position, failMode="standalone"))
                info("*** Creating links\n")
                self.addLink(robots[idx], aps, cls=TCLink)
            elif node_type[idx] == "MOBILE":
                robots.append(self.addStation(node, ip=ip_list[idx], cls=DockerSta, dimage="cornet:focalfoxyNWH",
                                              cpu_shares=20, position=position))
                #aps.append(0)
        # gazebo=self.addHost('gazebo', ip='10.0.0.252/8')
        # self.addLink(gazebo,aps.pop())
        #self.addLink(aps.pop(), robots.pop())
        # for idx, robot in enumerate(robots):
        #     self.addLink(robot, aps, cls=TCLink)
        #print (robots, aps)

def main(args):
    if len(args) != 2:
        print("usage: network_coordinator.py <config_file>")
    else:
        net = Containernet(topo=Topology_MN(args[1]), link=wmediumd,
                           wmediumd_mode=interference)
        net.setPropagationModel(model="logDistance", exp=4)
        info("*** Configuring wifi nodes\n")
        #net.configureWifiNodes()
        if '-p' not in args:
            net.plotGraph(max_x=100, max_y=100)
        #nodes = net.stations  # + net.aps
        #net.telemetry(nodes=nodes, single=True, max_x=1000, max_y=1000, max_z=1000, data_type='position')

        info("*** Starting Network\n")
        # net.addNAT(linkTo='ap1').configDefault()
        net.start()
        aps = net.get('ap1')
        info("*** printing all nodes\n")
        print (aps)
        info("*** Staring Socket Server\n")
        aps.start([])

        net.socketServer(ip='127.0.0.1', port=12345)

        info("*** Running CLI\n")
        CLI(net)

        os.system('sudo service network-manager start')
        info("*** Stopping network\n")
        net.stop()


if __name__ == '__main__':
    os.system('sudo service network-manager stop')
    setLogLevel('debug')
    sys.exit(main(sys.argv))
