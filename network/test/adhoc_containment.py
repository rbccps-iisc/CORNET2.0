#!/usr/bin/python

"""
This example shows on how to enable the adhoc mode
Alternatively, you can use the manet routing protocol of your choice
"""

import sys

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

    # sta1 = net.addStation('sta1', ip6='fe80::1',
    #                       position='260.0,0.0,0', **kwargs)
    # sta2 = net.addStation('sta2', ip6='fe80::2',
    #                       position='150.0,-230.0,0', **kwargs)
    # sta3 = net.addStation('sta3', ip6='fe80::3',
    #                       position='30.0,40.0,0', **kwargs)
    # sta4 = net.addStation('sta4', ip6='fe80::3',
    #                       position='179,-3.0,0', **kwargs)
    # sta5 = net.addStation('sta5', ip6='fe80::3',
    #                       position='126,-116,0', **kwargs)
    # sta6 = net.addStation('sta6', ip6='fe80::3',
    #                       position='108,-23,0', **kwargs)
    sta1 = net.addStation('sta1', ip6='fe80::1',
                          position='260.0,0.0,0', **kwargs)
    sta2 = net.addStation('sta2', ip6='fe80::2',
                          position='150.0,-230.0,0', **kwargs)
    sta3 = net.addStation('sta3', ip6='fe80::3',
                          position='30.0,40.0,0', **kwargs)
    sta4 = net.addStation('sta4', ip6='fe80::3',
                          position='-90.0,40.0,0', **kwargs)
    sta5 = net.addStation('sta5', ip6='fe80::3',
                          position='280.0,160.0,0', **kwargs)
    sta6 = net.addStation('sta6', ip6='fe80::3',
                          position='-10.0,120.0,0', **kwargs)

    net.setPropagationModel(model="logDistance", exp=3.5)

    info("*** Configuring wifi nodes\n")
    net.configureWifiNodes()

    if '-p' not in args:
        net.plotGraph(min_x=-100, min_y=-250, max_x=300, max_y=180)

    info("*** Creating links\n")
    # MANET routing protocols supported by proto:
    # babel, batman_adv, batmand and olsr
    # WARNING: we may need to stop Network Manager if you want
    # to work with babel
    protocols = ['babel', 'batman_adv', 'batmand', 'olsrd', 'olsrd2']
    kwargs = dict()
    for proto in args:
        if proto in protocols:
            kwargs['proto'] = proto

    net.addLink(sta1, cls=adhoc, intf='sta1-wlan0',
                ssid='adhocNet', mode='g', channel=5,
                ht_cap='HT40+', **kwargs)
    net.addLink(sta2, cls=adhoc, intf='sta2-wlan0',
                ssid='adhocNet', mode='g', channel=5,
                **kwargs)
    net.addLink(sta3, cls=adhoc, intf='sta3-wlan0',
                ssid='adhocNet', mode='g', channel=5,
                ht_cap='HT40+', **kwargs)
    net.addLink(sta4, cls=adhoc, intf='sta4-wlan0',
                ssid='adhocNet', mode='g', channel=5,
                ht_cap='HT40+', **kwargs)
    net.addLink(sta5, cls=adhoc, intf='sta5-wlan0',
                ssid='adhocNet', mode='g', channel=5,
                ht_cap='HT40+', **kwargs)
    net.addLink(sta6, cls=adhoc, intf='sta6-wlan0',
                ssid='adhocNet', mode='g', channel=5,
                ht_cap='HT40+', **kwargs)
    info("*** Starting network\n")
    net.build()

    info("\n*** Addressing...\n")
    if 'proto' not in kwargs:
        sta1.setIP6('2001::1/64', intf="sta1-wlan0")
        sta2.setIP6('2001::2/64', intf="sta2-wlan0")
        sta3.setIP6('2001::3/64', intf="sta3-wlan0")
        sta4.setIP6('2001::4/64', intf="sta4-wlan0")
        sta5.setIP6('2001::5/64', intf="sta5-wlan0")
        sta6.setIP6('2001::6/64', intf="sta6-wlan0")

    info("*** Running CLI\n")
    CLI(net)

    info("*** Stopping network\n")
    net.stop()


if __name__ == '__main__':
    setLogLevel('info')
    topology(sys.argv)
