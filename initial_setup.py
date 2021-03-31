import rospy
from gazebo_msgs.srv import GetModelState
import yaml


class CornetConfig:
    def __init__(self,config):
        self.pose = []



        for idx, model_name in enumerate(models):
            self.pose[idx] = self.get_robot_info(model_name)

    def get_robot_info(self, model_name):
        # rospy.init_node('check_odometry')
        relative_entity_name = ''
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            return gms(model_name, relative_entity_name)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

def main(args):
    if len(args) != 2:
        print("usage: network_coordinator.py <config_file>")
    else:

        CornetConfig(args[1])