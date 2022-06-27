#!/usr/bin/python
"""
add
"""
from containernet.net import Containernet
from containernet.node import DockerSta, Docker
from containernet.cli import CLI
from containernet.term import makeTerm
from mininet.log import info, setLogLevel
from mn_wifi.link import wmediumd
from mn_wifi.wmediumdConnector import interference
import sys
import os
import yaml


def topology(args):
    net = Containernet(link=wmediumd, wmediumd_mode=interference, noise_th=-91, fading_cof=3)
    if len(args) != 2:
        print("usage: network_config.py <config_file>")
    else:

        with open(args[1]) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        models = config['gazebo_models']
        node_type = config['type']
        pose = config['pose']
        ip_list = config['ip_list']
        image = config['image']
        string1 = ""
        string1 = string1.join(image)
        #print(string1)
        # print(imag)
        sta_list = []
        ap_list = []
        info('*** Adding docker containers\n')
        for idx, node in enumerate(models):
            position = str(pose[idx]['position']['x']) + "," + str(pose[idx]['position']['y']) + "," + str(
                pose[idx]['position']['z'])
            #print(type(node), type(ip_list[idx]))

            if node_type[idx] == "STATIC":
                #sta_list.append(net.addStation('host%s' % idx, ip=ip_list[idx], mac='00:02:00:00:00:1%s' % idx,
                #                              cls=DockerSta, dimage=string1, cpu_shares=20, position=position))
                ap = idx%4
                ap_list.append(net.addAccessPoint(node, ssid='new-ssid%s' %ap, mode='g', position=position,
                                                  failMode="standalone"))
                #FIXME add channel parameters as well to the config file.

                #net.addLink(sta_list[idx], ap_list[idx])#, cls=TCLink)
            elif node_type[idx] == "MOBILE":
                sta_list.append(net.addStation(node, ip=ip_list[idx],
                                               cls=DockerSta, dimage=string1, cpu_shares=20, position=position))
                #ap_list.append(0)

        #T_op= net.addStation('tele', ip='10.0.0.1', mac='00:02:00:00:00:20',
        #                 cls=DockerSta, dimage="cornet:focalfoxyNWH", cpu_shares=20, position='2,10,0')
        #c0 = net.addController('c0')
        h1 = net.addHost('h1', ip='10.0.0.1/24', cls=Docker, dimage="cornet:focalfoxyNWH", cpu_shares=20)

        info("*** Configuring Propagation Model\n")
        net.setPropagationModel(model="logDistance", exp=6.5)
        # FIXME add propagation model as well to the config file.

        #info('*** Adding switches\n')
        #s1 = net.addSwitch('s1')

        #for ap in ap_list:
        #    net.addLink(s1 , ap)

        info('*** Configuring WiFi nodes\n')
        net.configureWifiNodes()

        if '-p' not in args:
            net.plotGraph(max_x=100, max_y=100)

        for sta in sta_list:
            sta.cmd('service ssh restart')

        h1.cmd('service ssh restart')

        info('*** Starting network\n')
        net.build()
        for ap in ap_list:
            #if ap != 0:
            ap.start([])
            ap.cmd("ovs-ofctl add-flow %s priority=1,arp,actions=flood" % ap)


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
