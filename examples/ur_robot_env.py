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

    def home_position(self):
        home_pose = (1.5956498, -1.47229, 1.4730, -1.60006, 4.688987, 0.07035151) # Joint in rad for home position
        self.robot.movej(home_pose, vel=self.vel, acc=self.acc, wait=True)
        logging.info("Robot at home")


    def read_joint_state(self):
        joint_state = self.robot.getj()
        return joint_state

    def move_reset_position(self):
        reset_pose = (1.595745682, -1.5322321, 1.4995, -3.11227, 4.724702, 0.0004789)
        self.robot.movej(reset_pose, vel=self.vel, acc=self.acc, wait=True)
        logging.info("Robot at reset pose")



def read_config_file():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, '..', 'config', 'robot_config.json')
    with open(config_path) as f:
        config = json.load(f)
    return config

def main():
    config = read_config_file()
    ip_address_robot = config['ip_robot']

    # robot = urx.Robot(ip_address_robot, use_rt=True, urFirm=config['urFirm'])
    robot = urx.Robot(ip_address_robot)
    

    robot_env = Env(robot)
    joints = robot_env.read_joint_state()

    robot_env.home_position()
    robot_env.move_reset_position()

    robot.close()


if __name__ == "__main__":
    main()
