import os
import json
import urx
import logging
logging.basicConfig(level=logging.INFO)
import collections
collections.Iterable = collections.abc.Iterable # Need this for math3d lib issues
from environment.main_environment_ur5 import Environment


def read_config_file():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, 'config', 'robot_config.json')
    print(config_path)
    with open(config_path) as f:
        config = json.load(f)
    return config


def main():
    config = read_config_file()
    ip_address_robot = config['ip_robot']

    # robot = urx.Robot(ip_address_robot, use_rt=True, urFirm=config['urFirm'])
    robot = urx.Robot(ip_address_robot)


    env = Environment(robot, velocity=config['velocity'], acceleration=config['acceleration'])
    env.starting_position() # just making sure the joint are in the right position for initialization
    env.robot_home_position()

    for i in range(1):
        env.hard_code_solution()

    env.robot_home_position()


    robot.close()


if __name__ == "__main__":
    main()
