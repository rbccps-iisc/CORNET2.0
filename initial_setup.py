import sys
import rospy
from gazebo_msgs.srv import GetModelState
import yaml


class CornetConfig:
    def __init__(self, config_file):
        self.gen_config = {}
        self.pose = []

        with open(config_file) as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

    def get_robot_info(self, model_name):
        # rospy.init_node('check_odometry')
        resp = {}
        res = {}
        relative_entity_name = ''
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            point = gms(model_name, relative_entity_name)
            resp['x'] = int(point.pose.position.x)
            resp['y'] = int(point.pose.position.y)
            resp['z'] = int(point.pose.position.z)
            res['position'] = resp
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def generate_config(self):
        models = self.config['gazebo_models']
        #for idx, model_name in enumerate(models):
        #    p = self.get_robot_info(model_name)
        #    self.pose.append(p)

        #self.config['pose'] = self.pose
        print(self.gen_config)
        for item in self.gen_config:
            print(item)
        for val in self.config['ip_list']:
            print type(val)



        # print (self.gen_config)
        with open('config.yaml', 'w') as file:
            documents = yaml.dump(self.config, file)


def main(args):
    if len(args) != 2:
        print("usage: network_coordinator.py <config_file>")
    else:
        conf = CornetConfig(args[1])
        conf.generate_config()


if __name__ == '__main__':
    sys.exit(main(sys.argv))
