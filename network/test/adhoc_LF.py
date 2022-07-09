import sys
import os
import subprocess

from mininet.log import setLogLevel, info
from mn_wifi.link import wmediumd, adhoc
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mn_wifi.wmediumdConnector import interference
from mn_wifi.telemetry import util_dir
from mn_wifi.node import AP
import mn_wifi
import time

start = time.time()
from threading import Thread as thread


class Stats:
    def __init__(self, nodes):
        self.nodes = nodes
        self.filename = '{}-mn-telemetry.txt'
        # self.stats_file = '{}-mn-stats.txt'
        self.thread_ = None
        # self.dir = 'cat /sys/class/ieee80211/{}/device/net/{}/statistics/{}'
        # self.data_type = 'tx_bytes'

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

    def get_ping(self, node):
        cmd = '~/ping_s.sh 10.0.0.1'
        isAP = False

        if isinstance(node, AP) and not isAP:
            result= subprocess.check_output(cmd,
                                            shell=True, executable='/bin/bash').decode()#.split("\n")
            isAP = True
        else:
            if not isinstance(node, AP):
                result = subprocess.check_output('%s %s %s' % (util_dir, node, cmd),
                                                shell=True, executable='/bin/bash').decode()#.split("\n")
        return result

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
                    ping = self.get_ping(node)
                    os.system("echo '%s, %s, %s, %s, %s' >> %s" % (time_, x, y, rssi, ping, self.filename.format(node)))


def topology(args):
    "Create a network."
    net = Mininet_wifi(link=wmediumd, wmediumd_mode=interference)

    info("*** Creating nodes\n")
    kwargs = dict()
    if '-a' in args:
        kwargs['range'] = 100

    sta1 = net.addStation('sta1', ip6='fe80::1',
                          position='10,10,0', **kwargs)
    sta2 = net.addStation('sta2', ip6='fe80::2',
                          position='40,10,0', **kwargs)
    # sta3 = net.addStation('sta3', ip6='fe80::3',
    #                       position='60,10,0', **kwargs)

    net.setPropagationModel(model="logDistance", exp=4)

    info("*** Configuring wifi nodes\n")
    net.configureWifiNodes()

    # if '-p' not in args:
    #     net.plotGraph(max_x=100, max_y=100)
    nodes = net.stations + net.aps
    net.telemetry(nodes=nodes, single=True, min_x=-100, min_y=-100, max_x=100, max_y=100, data_type='position')

    # stat = Stats(nodes)
    # stat.thread_ = thread(target=stat.start)
    # stat.thread_.daemon = True
    # stat.thread_._keep_alive = True
    # stat.thread_.start()

    info("*** Creating links\n")
    # MANET routing protocols supported by proto:
    # babel, batman_adv, batmand and olsr
    # WARNING: we may need to stop Network Manager if you want
    # to work with babel
    # protocols = ['babel', 'batman_adv', 'batmand', 'olsrd', 'olsrd2']
    # kwargs = dict()
    # for proto in args:
    #     if proto in protocols:
    #         kwargs['proto'] = proto

    net.addLink(sta1, cls=adhoc, intf='sta1-wlan0',
                ssid='adhocNet', mode='g', channel=5, proto='batman_adv',
                ht_cap='HT40+', **kwargs)
    net.addLink(sta2, cls=adhoc, intf='sta2-wlan0',
                ssid='adhocNet', mode='g', channel=5, proto='batman_adv',
                **kwargs)
    # net.addLink(sta3, cls=adhoc, intf='sta3-wlan0',
    #             ssid='adhocNet', mode='g', channel=5, proto='batman_adv',
    #             ht_cap='HT40+', **kwargs)

    info("*** Starting network\n")
    net.build()

    # info("\n*** Addressing...\n")
    # if 'proto' not in kwargs:
    #     sta1.setIP6('2001::1/64', intf="sta1-wlan0")
    #     sta2.setIP6('2001::2/64', intf="sta2-wlan0")
    #     sta3.setIP6('2001::3/64', intf="sta3-wlan0")

    net.socketServer(ip='127.0.0.1', port=12345)

    info("*** Running CLI\n")
    CLI(net)

    os.system('sudo service network-manager start')
    info("*** Stopping network\n")
    net.stop()


if __name__ == '__main__':
    os.system('sudo systemctl stop network-manager')
    setLogLevel('info')
    topology(sys.argv)
