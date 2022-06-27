#!/usr/bin/python
"""
add
"""
from containernet.net import Containernet
from containernet.node import DockerSta, Docker
from containernet.cli import CLI
from containernet.term import makeTerm
from containernet.link import TCLink
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
                                           ip=robot_information["robot_ip"][robot_no], cls=DockerSta,
                                           dimage=string1, position=position))

        for ap_no in range(0, num_aps):
            position = str(str(static_information["ap_pos_x"][ap_no]) + "," + str(static_information["ap_pos_y"][ap_no])
                           + "," + str(static_information["ap_pos_x"][ap_no]))
            ap_list.append(net.addAccessPoint(static_information["ap_namespace"][ap_no], ssid='new-ssid%s' % ap_no,
                                              mode='g', position=position, failMode="standalone"))

        # c0 = net.addController('c0')
        h1 = net.addHost('h1', ip='10.0.0.1/24', cls=Docker, dimage="cornet:focalfoxyNWH")

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
            #net.plotGraph(min_x=-15, max_x=15, min_y=-15, max_y=15)
	     net.plotGraph(max_x=50, max_y=50)

        for sta in sta_list:
            sta.cmd('service ssh restart')

        h1.cmd('service ssh restart')

        info('*** Starting network\n')
        net.build()

        for ap in ap_list:
            # if ap != 0:
            ap.start([])
            ap.cmd("ovs-ofctl add-flow %s priority=1,arp,actions=flood" % ap)
        # s1.start([])
        #ap1.start([])
        #ap1.cmd("ovs-ofctl add-flow ap1 priority=1,arp,actions=flood")
        # s1.cmd("ovs-ofctl add-flow s1 priority=1,arp,actions=flood")

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
