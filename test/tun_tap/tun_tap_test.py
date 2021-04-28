import subprocess

import ipaddress as ipaddress
import fcntl
import struct
import select
import os
import sys

import multiprocessing as multiprocessing
import threading

ip_list = ['10.8.0.1', '10.8.0.2']  # ,"10.8.0.3","10.8.0.4"]
num_ips = len(ip_list)

TUNSETIFF = 0x400454ca
TUNSETOWNER = TUNSETIFF + 2
IFF_TUN = 0x0001
# IFF_TAP = 0x0002
IFF_NO_PI = 0x1000
run_event = multiprocessing.Event()
tuns = []
tun_threads = []


def _setup_network(ip_list):
    for i, ip_i in enumerate(ip_list):
        # ~ sudo ip tuntap add dev tun0 mode tun
        # ~ sudo ip link set tun0 up
        # ~ sudo ip addr add 192.168.0.1/32 dev tun0
        subprocess.call(["sudo", "ip", "tuntap", "add", "dev", "tun" + str(i), "mode", "tun"])
        subprocess.call(["sudo", "ip", "link", "set", "tun" + str(i), "up"])
        subprocess.call(["sudo", "ip", "addr", "add", ip_i + "/32", "dev", "tun" + str(i)])

        # ~ sudo ip route add 192.168.0.2/32 dev tun0 src 192.168.0.1 table 1
        # ~ sudo ip rule add table 1 from 192.168.0.1 priority 2
        for j, ip_j in enumerate(ip_list):
            if j != i:
                subprocess.call(["sudo", "ip", "route", "add", ip_j + "/32", "dev", "tun" + str(i),
                                 "src", ip_i, "table", str(i + 1)])
        subprocess.call(["sudo", "ip", "rule", "add", "table", str(i + 1), "from", ip_i, "priority", "2"])

        # ~ #Conditional local
        # ~ sudo ip rule add iif tun0 lookup 101 priority 1
        # ~ sudo ip route add local 192.168.0.1 dev tun0 table 101
        subprocess.call(["sudo", "ip", "rule", "add", "iif", "tun" + str(i), "table", str(i + 101), "priority", "1"])
        subprocess.call(["sudo", "ip", "route", "add", "local", ip_i, "dev", "tun" + str(i), "table", str(i + 101)])

    # ~ sudo ip rule del pref 0 from all lookup local
    # ~ sudo ip rule add pref 10 from all lookup local
    subprocess.call(["sudo", "ip", "rule", "del", "pref", "0", "from", "all", "lookup", "local"])
    subprocess.call(["sudo", "ip", "rule", "add", "pref", "10", "from", "all", "lookup", "local"])


def _remove_network(num_ips):
    for i in range(num_ips):
        # ~ sudo ip tuntap del dev tun0 mode tun
        # ~ sudo ip rule del table 1
        # ~ sudo ip rule del table 101
        subprocess.call(["sudo", "ip", "tuntap", "del", "dev", "tun" + str(i), "mode", "tun"])
        subprocess.call(["sudo", "ip", "rule", "del", "table", str(i + 1)])
        subprocess.call(["sudo", "ip", "rule", "del", "table", str(i + 101)])

    # ~ sudo ip rule add pref 0 from all lookup local
    # ~ sudo ip rule del pref 10 from all lookup local
    subprocess.call(["sudo", "ip", "rule", "add", "pref", "0", "from", "all", "lookup", "local"])
    subprocess.call(["sudo", "ip", "rule", "del", "pref", "10", "from", "all", "lookup", "local"])

def _read_from_tuns( i, tuns):
    while run_event.is_set():
        # read from TUN
        try:
            r, __, __ = select.select([tuns[i].fileno(), ], [], [], 2)
            if r:
                data = os.read(tuns[i].fileno(), 4096)
            else:
                continue
        except OSError:
            print("TUN " + str(i) + " seems to be gone, can't read from it")
            break

        # identify IPs and save into buffer
        try:
            print type(data)
            print data[0]#, data.encode('hex')

            version = data[0]
            print ("version ",type(version))
            #version = version >> 4
            #if version == 4:
            #   ip_src = int.from_bytes(data[12:16], byteorder="big")
            #    ip_dst = int.from_bytes(data[16:20], byteorder="big")
            #   print(ip_src, ip_dst, data)
        except ValueError, e:
            print e



    print("TUN " + str(i) + " exiting")
    tuns[i].close()

if str(sys.argv[1])== 'start':
    print "start"
    _setup_network(ip_list)

    ip_to_tun_map = {int(ipaddress.IPv4Address(unicode(ip_i))): i for i, ip_i in enumerate(ip_list)}

    # open TUNs
    for i in range(num_ips):
        tuns.append(open('/dev/net/tun', 'r+b', buffering=0))
        ifri = struct.pack('16sH', b'tun' + str(i).encode('ascii'), IFF_TUN | IFF_NO_PI)
        fcntl.ioctl(tuns[i], TUNSETIFF, ifri)
        fcntl.ioctl(tuns[i], TUNSETOWNER, 1000)

    run_event.set()
    # start TUN threads
    for i in range(num_ips):
        tun_threads.append(threading.Thread(target=_read_from_tuns, args=(i, tuns,)))
        tun_threads[i].start()



elif str(sys.argv[1]) =='stop':
    _remove_network(len(ip_list))

else:
    print "nothing"

print 'Argument List:', str(sys.argv)
