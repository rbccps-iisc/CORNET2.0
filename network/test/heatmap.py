"""Setting position of the nodes and enable sockets"""
import subprocess
import sys
import mn_wifi
from mininet.log import setLogLevel, info
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mn_wifi.telemetry import util_dir
from mn_wifi.node import AP
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference
from mn_wifi.telemetry import util_dir
from mn_wifi.node import AP
import sys
import os
import yaml
import time
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference
start = time.time()
from threading import Thread as thread

class Stats:
    def __init__(self, nodes):
        self.nodes = nodes
        self.filename = '{}-mn-telemetry.txt'
        self.thread_ = None
    @classmethod
    def get_ifaces(cls, nodes, inNamespaceNodes, phys):
        cmd = 'ls /sys/class/ieee80211/{}/device/net/'
        ifaces = {}
        j = 0
        wlan = 1
        for phy in phys:
            try:
                ifaces_ = subprocess.check_output(cmd.format(phy), stderr=subprocess.PIPE,
                                                  shell=True).decode().split("\n")
            except:
                node = inNamespaceNodes[j]
                ifaces_ = subprocess.check_output('%s %s %s' % (util_dir, node, cmd.format(phy)),
                                                  shell=True).decode().split("\n")
            ifaces_.pop()
            if nodes[j] not in ifaces:
                ifaces[nodes[j]] = []
            ifaces[nodes[j]].append(ifaces_.pop())

            if wlan == len(nodes[j].params['wlan']):
                j += 1
                wlan = 1
            else:
                wlan += 1

        return ifaces

    @classmethod
    def get_phys(cls, nodes, inNamespaceNodes):
        cmd = 'ls /sys/class/ieee80211/'
        isAP = False
        phys = []
        for node in nodes:
            if isinstance(node, AP) and not isAP:
                phys += subprocess.check_output(cmd,
                                                shell=True).decode().split("\n")
                isAP = True
            else:
                if not isinstance(node, AP):
                    phys += subprocess.check_output('%s %s %s' % (util_dir, node, cmd),
                                                    shell=True).decode().split("\n")
        phy_list = []
        phys = sorted(phys)
        for phy in phys:
            if 'mn' in phy:
                phy_list.append(phy)

        ifaces = cls.get_ifaces(nodes, inNamespaceNodes, phy_list)
        return phy_list, ifaces

    def start(self):

        inNamespaceNodes = []
        nodes_x = {}
        nodes_y = {}
        names = []
        time_ = time.time() - start
        for node in self.nodes:
            if not isinstance(node, AP):
                inNamespaceNodes.append(node)
            if os.path.exists('%s' % (self.filename.format(node))):
                os.system('rm %s' % (self.filename.format(node)))
            if node.name not in names:
                names.append(node.name)

        self.phys, self.ifaces = self.get_phys(self.nodes, inNamespaceNodes)

        while self.thread_._keep_alive:
            for node in self.nodes:
                x, y, z = mn_wifi.telemetry.get_position(node)
                for wlan in range(len(node.params['wlan'])):
                    names.append(self.ifaces[node][wlan])
                    nodes_x[node], nodes_y[node] = [], []
                    rssi = mn_wifi.telemetry.get_rssi(node, self.ifaces[node][wlan])
                    os.system("echo '%s, %s, %s, %s' >> %s" % (time_, x, y, rssi, self.filename.format(node)))


def topology(args):
    net = Mininet_wifi(link=wmediumd, wmediumd_mode=interference, noise_th=-91, fading_cof=3)
    if len(args) != 2:
        print("usage: network_config.py <config_file>")
    else:
        with open(args[1]) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        num_robots = config['robot_number']
        robot_information = config['robot_information']

        num_aps = config['access_points']
        static_information = config['static_information']
        image = config['image']
        string1 = ""
        string1 = string1.join(image)

        sta_list = []
        ap_list = []
        info('*** Adding docker containers\n')
        for robot_no in range(0, num_robots):
            position = str(
                str(robot_information["robot_pos_x"][robot_no]) + "," + str(robot_information["robot_pos_y"][robot_no])
                + "," + str(robot_information["robot_pos_z"][robot_no]))
            sta_list.append(net.addStation(robot_information["robot_namespace"][robot_no],
                                           ip=robot_information["robot_ip"][robot_no], position=position))

        for ap_no in range(0, num_aps):
            position = str(str(static_information["ap_pos_x"][ap_no]) + "," + str(static_information["ap_pos_y"][ap_no])
                           + "," + str(static_information["ap_pos_x"][ap_no]))
            ap_list.append(net.addAccessPoint(static_information["ap_namespace"][ap_no], ssid='new-ssid%s' % ap_no,
                                              mode='g', position=position, failMode="standalone"))

        # c0 = net.addController('c0')
        #h1 = net.addHost('h1', ip='10.0.0.1/24', cls=Docker, dimage="cornet:focalfoxyNWH")

        #ap1 = net.addAccessPoint("ap1",ssid='centre', mode='g', position='30,30,0',failMode="standalone")


        info("*** Configuring Propagation Model\n")
        net.setPropagationModel(model="logDistance", exp=6)
        #FIXME add propagation model as well to the config file.

        # info('*** Adding switches\n')
        # s1 = net.addSwitch('s1')
        #ap1 = ap_list[0]
        # for idx, ap in enumerate(ap_list):
        #     #if idx != 0:
        #     net.addLink(ap1, ap, cls=TCLink)

        info('*** Configuring WiFi nodes\n')
        net.configureWifiNodes()

        if '-p' not in args:
            net.plotGraph(max_x=300, max_y=300)

        net.setMobilityModel(time=0, model='RandomDirection',
                             max_x=50, max_y=50, seed=20)

        nodes = net.stations  # + net.aps
        # net.telemetry(nodes=nodes, single=True, max_x=100, max_y=100, data_type='position')
        stat = Stats(nodes)
        stat.thread_ = thread(target=stat.start)
        stat.thread_.daemon = True
        stat.thread_._keep_alive = True
        stat.thread_.start()

        info('*** Starting network\n')
        net.build()

        for ap in ap_list:
            # if ap != 0:
            ap.start([])
            ap.cmd("ovs-ofctl add-flow %s priority=1,arp,actions=flood" % ap)
        #s1.start([])
        #ap1.start([])
        #ap1.cmd("ovs-ofctl add-flow ap1 priority=1,arp,actions=flood")
        # s1.cmd("ovs-ofctl add-flow s1 priority=1,arp,actions=flood")

        #net.socketServer(ip='127.0.0.1', port=12345)

        info('*** Running CLI\n')
        CLI(net)
        os.system('sudo service network-manager start')

        info('*** Stopping network\n')
        net.stop()


if __name__ == '__main__':
    os.system('sudo systemctl stop network-manager')
    setLogLevel('info')
    topology(sys.argv)
