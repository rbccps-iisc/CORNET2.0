import socket
import sys
import time
import logging
import yaml
from subprocess import check_output, CalledProcessError, STDOUT

logging.basicConfig(filename='gz_robot_position.log', encoding='utf-8', level=logging.DEBUG)


class GzClient:
    def __init__(self, config_file):

        # load the contents from the yaml config file.
        with open(config_file) as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

        # gazebo gz call configuration requriments
        self.models = self.config['gazebo_models']
        self.type = self.config['type']

        self.cmd = ["gz", "model", "-m", "robot", "-p"]

        # socker server connection to mininet-wifi/containernet
        self.host = '127.0.0.1'
        self.port = 12345
        self.msg = ["set.","station",".setPosition","(", "\"", "'", "xPosition", ",", "yPosition", ",", "zPosition", "\"", ")", "'"]

    def system_call(self, cmd):
        """
        params:
            command: list of strings, ex. `["ls", "-l"]`
        returns: output, success
        """
        try:
            output = check_output(cmd, stderr=STDOUT).decode()
            success = True
        except CalledProcessError as e:
            output = e.output.decode()
            success = False
        return output, success

    def get_robot_info(self):
        try:
            logging.info("get_robot_info")
            for idx, robot in enumerate(self.models):
                if self.type[idx] == "MOBILE":
                    self.cmd[3] = robot
                    logging.debug(self.cmd)
                    result, success = self.system_call(self.cmd)
                    resp = result.split(" ")
                    logging.debug(resp)

                    self.msg[1] = robot
                    self.msg[6] = resp[0].decode('utf-8')
                    self.msg[8] = resp[1].decode('utf-8')
                    self.msg[10] = resp[2].decode('utf-8')

                    logging.debug(''.join([str(item) for item in self.msg]))
                    #FIXME the update to the mininet server is pending
                    result = self.client(''.join([str(item) for item in self.msg]))
                    logging.debug(result)
                    time.sleep(1)
        except Exception, e:
            print " failed: %s" % e

    def client(self, msg):
        # while msg != 'q' and msg != 'exit':
        s = socket.socket()
        s.connect((self.host, self.port))
        s.send(str(msg).encode('utf-8'))
        data = s.recv(1024).decode('utf-8')
        logging.debug("Recv from Server")
        s.close()
        return data


def main(args):
    if len(args) != 2:
        print("usage: gz_robot_position.py <config_file>")
    else:
        conf = GzClient(args[1])
        while True:
            try:
                conf.get_robot_info()
            except KeyboardInterrupt:
                break


if __name__ == '__main__':
    sys.exit(main(sys.argv))
