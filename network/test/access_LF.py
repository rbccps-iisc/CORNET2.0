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
                          position='0,0,0', **kwargs)
    sta2 = net.addStation('sta2', ip6='fe80::2',
                          position='5,5,0', **kwargs)
    sta3 = net.addStation('sta3', ip6='fe80::3',
                          position='5,-5,0', **kwargs)

    ap1 = net.addAccessPoint('ap1', ssid='new-ssid', mode='a', channel='36', position='1,10,0')
    net.setPropagationModel(model="logDistance", exp=4)
    info("*** Configuring wifi nodes\n")
    net.configureWifiNodes()

    info("*** Creating links\n")

    nodes = net.stations + net.aps
    net.telemetry(nodes=nodes, single=True,min_x=-100, min_y=-100, max_x=100, max_y=100, data_type='position')

    info('*** Starting network\n')
    net.build()
    ap1.start([])

    info("*** Staring Socket Server\n")

    net.socketServer(ip='127.0.0.1', port=12345)

    info('*** Running CLI\n')
    CLI(net)
    os.system('sudo service network-manager start')

    info('*** Stopping network\n')
    net.stop()


if __name__ == '__main__':
    os.system('sudo systemctl stop network-manager')
    setLogLevel('info')
    topology(sys.argv)
