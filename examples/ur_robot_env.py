import os
import json
import urx
from math import pi
import logging
logging.basicConfig(level=logging.INFO)
import collections
collections.Iterable = collections.abc.Iterable # Need this for math3d lib issues



class Env:
    def __init__(self, robot):
        self.robot = robot
        self.initial_set_up()
        self.vel = 0.1
        self.acc = 0.8

    def initial_set_up(self):
        self.robot.set_tcp((0, 0, 0, 0, 0, 0))  # Set tool central point
        self.robot.set_payload(0.5, (0, 0, 0))  # Kg

    def home_position(self, joint_state):
        #home_pose = (-1.600, -1.727, -2.202, -0.807, 1.595, -0.030) # Joint in rad for home position
        self.robot.movej(joint_state, vel=self.vel, acc=self.acc, wait=True)
        logging.info("Robot at home")

    def read_joint_state(self):
        joint_state = self.robot.getj()
        return joint_state

    # def move_rest_position(self):
    #     self.robot.movel





def read_config_file():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, '..', 'config', 'robot_config.json')
    with open(config_path) as f:
        config = json.load(f)
    return config

def main():
    config = read_config_file()
    ip_address_robot = config['ip_robot']

    robot = urx.Robot(ip_address_robot, use_rt=True, urFirm=config['urFirm'])
    robot_env = Env(robot)

    joints = robot_env.read_joint_state()
    print(joints)

    #robot_env.home_position()

    robot.close()


if __name__ == "__main__":
    main()
