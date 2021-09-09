import re
import shlex
import subprocess
import time
from random import randint


def main():
    '''
        Script to continuosly publish ping results at random time intervals to update cost-map of the robots
    '''

    while (True):
        cmd = shlex.split(
            'ansible-playbook -i ./hosts pingtask.yml')
        process = subprocess.Popen(cmd,
                                   stdout=subprocess.PIPE,
                                   universal_newlines=True)
        stdout, stderr = process.communicate()
        # print(stdout)
        # print(stderr)

        filter = re.findall(r'result.stdout": \[[^\]]*\]', stdout)
        robot_filter = re.findall(r'ok: \[[^\]]*\] =>', stdout)
        #print (robot_filter)

        robots = ['172.17.0.2', '172.17.0.3', '172.17.0.4', '172.17.0.5']
        #FIXME: get from hosts
        a = []
        result = []
        for i in filter:
            a = re.split('(true)|(false)', i)
            i = 0
            while i < len(a):
                if a[i] not in ['true', 'false']:
                    a.pop(i)
                else:
                    i += 1
            if a != 0:
                result.append(a)
        cur_robotlist =[]
        for i in robot_filter:
            a = re.findall('\[[^\]]*\]', i)[0][1:-1]
            #print(a)
            cur_robotlist.append(a)

        output = {}
        output['allpings'] = []
        #for val in result:
        for i in range(len(result)) :
            temp = {}
            temp['ping'] = result[i]
            temp['robot_id'] = robots.index(cur_robotlist[i])
            output['allpings'] += [temp]
        output = str(output)
        output = output.replace("'","")
        #print (output)
        cmd1 = shlex.split(
            'ros2 topic pub --once /ping warehouse_interfaces/msg/PingStatus "' + output + '"')
        #print (cmd)
        process = subprocess.Popen(cmd1,
                                   stdout=subprocess.PIPE,
                                   universal_newlines=True)
        stdout, stderr = process.communicate()
        print(stdout)
        print(stderr)
        # ros2 topic pub / ping
        # warehouse_interfaces / msg / PingStatus
        # '{allpings:[{ping:[true,false,false,true]}, {ping:[true,true,true,true]}, {ping:[true,true,true,false]}, {ping:[false,true,true,true]}]}'

        #time.sleep(1)


if __name__ == '__main__':
    main()
