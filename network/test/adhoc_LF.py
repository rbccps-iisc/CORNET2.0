import sys
import os
from mininet.log import setLogLevel, info
from mn_wifi.link import wmediumd, adhoc
from mn_wifi.cli import CLI
from mn_wifi.net import Mininet_wifi
from mn_wifi.wmediumdConnector import interference


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
    sta3 = net.addStation('sta3', ip6='fe80::3',
                          position='60,10,0', **kwargs)

    net.setPropagationModel(model="logDistance", exp=4.5)

    info("*** Configuring wifi nodes\n")
    net.configureWifiNodes()

    if '-p' not in args:
        net.plotGraph(max_x=100, max_y=100)

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
                ssid='adhocNet', mode='g', channel=5, proto='batmand',
                ht_cap='HT40+', **kwargs)
    net.addLink(sta2, cls=adhoc, intf='sta2-wlan0',
                ssid='adhocNet', mode='g', channel=5, proto='batmand',
                **kwargs)
    net.addLink(sta3, cls=adhoc, intf='sta3-wlan0',
                ssid='adhocNet', mode='g', channel=5, proto='batmand',
                ht_cap='HT40+', **kwargs)

    info("*** Starting network\n")
    net.build()

    info("\n*** Addressing...\n")
    if 'proto' not in kwargs:
        sta1.setIP6('2001::1/64', intf="sta1-wlan0")
        sta2.setIP6('2001::2/64', intf="sta2-wlan0")
        sta3.setIP6('2001::3/64', intf="sta3-wlan0")


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
