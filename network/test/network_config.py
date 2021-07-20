#!/usr/bin/python
"""
add
"""
from containernet.net import Containernet
from containernet.node import DockerSta
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
        print(string1)
        # print(imag)
        sta_list = []
        ap_list = []
        info('*** Adding docker containers\n')
        # for idx, node in enumerate(models):
        #     position = str(pose[idx]['position']['x']) + "," + str(pose[idx]['position']['y']) + "," + str(
        #         pose[idx]['position']['z'])
        #
        #     if node_type[idx] == "STATIC":
        #         # sta_list.append(net.addStation('host%s'%idx, ip=ip_list[idx],
        #         #                              cls=DockerSta, dimage=string1, cpu_shares=20))
        #         ap_list.append(net.addAccessPoint(node, ssid='new-ssid', mode='g', channel='6', position=position))
        #         # net.addLink(sta_list[idx], ap_list[idx], cls=TCLink)
        #     elif node_type[idx] == "MOBILE":
        #         sta_list.append(net.addStation(node, ip=ip_list[idx], mac='00:02:00:00:00:1%s' % idx,
        #                                        cls=DockerSta, dimage=string1, cpu_shares=20, position=position))
        info('*** Adding docker containers\n')
        sta1 = net.addStation('name_robo0', ip='10.0.0.1/8', mac='00:02:00:00:00:10',
                              cls=DockerSta, dimage=string1, cpu_shares=20, position='2,10,0')
        sta2 = net.addStation('name_robo1', ip='10.0.0.2/8', mac='00:02:00:00:00:11',
                              cls=DockerSta, dimage=string1, cpu_shares=20, position='10,10,0')
        ap1 = net.addAccessPoint('ap_0', ssid='new-ssid', mode='g', channel='6', position='1,1,0')

        c0 = net.addController('c0')

        info("*** Configuring Propagation Model\n")
        net.setPropagationModel(model="logDistance", exp=4)

        info('*** Configuring WiFi nodes\n')
        net.configureWifiNodes()

        if '-p' not in args:
            net.plotGraph(max_x=200, max_y=200)

        info('*** Starting network\n')
        net.build()
        for ap in ap_list:
            ap.start([c0])
        net.socketServer(ip='127.0.0.1', port=12345)

        info('*** Running CLI\n')
        CLI(net)
        os.system('sudo service network-manager start')

        info('*** Stopping network\n')
        net.stop()


if __name__ == '__main__':
    # os.system('sudo systemctl stop network-manager')
    setLogLevel('info')
    topology(sys.argv)
