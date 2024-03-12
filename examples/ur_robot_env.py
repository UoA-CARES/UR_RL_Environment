import os
import json
import urx
import math
import random
from math import pi
import logging
logging.basicConfig(level=logging.INFO)
import collections
collections.Iterable = collections.abc.Iterable # Need this for math3d lib issues
import math3d
import numpy as np

class Env:
    def __init__(self, robot):
        self.robot = robot
        self.initial_set_up()
        self.vel = 1.0
        self.acc = 3.0

        self.home_x = 0.14
        self.home_y = -0.50
        self.home_z = 0.40

        # wrist 3 range
        self.min_angle_deg = -40
        self.max_angle_deg = 40

        # ellipse area
        self.h, self.k = (self.home_x, self.home_z)  # central_point in (x, z)
        self.a = 0.30  # Semi-major axis length  x-axis
        self.b = 0.25  # Semi-minor axis length  z-axis


    def initial_set_up(self):
        self.robot.set_tcp((0, 0, 0, 0, 0, 0))  # Set tool central point
        self.robot.set_payload(0.4, (0, 0, 0))  # Kg

    def read_joint_state(self):
        joint_state = self.robot.getj()
        return joint_state

    def initial_position(self):
        home_pose = (math.radians(90), math.radians(-90), math.radians(90), math.radians(0), math.radians(90), math.radians(0))  # Joint in rad for home position
        self.robot.movej(home_pose, vel=self.vel, acc=self.acc, wait=True)
        logging.info("Robot at initial position")


    def RPYtoRotVec(self, angles):
        roll, pitch, yaw = angles
        roll, pitch, yaw = math.radians(roll), math.radians(pitch), math.radians(yaw)

        yawMatrix   = np.matrix([[math.cos(yaw), -math.sin(yaw), 0],[math.sin(yaw), math.cos(yaw), 0],[0, 0, 1]])
        pitchMatrix = np.matrix([[math.cos(pitch), 0, math.sin(pitch)],[0, 1, 0],[-math.sin(pitch), 0, math.cos(pitch)]])
        rollMatrix  = np.matrix([[1, 0, 0],[0, math.cos(roll), -math.sin(roll)],[0, math.sin(roll), math.cos(roll)]])

        R = yawMatrix * pitchMatrix * rollMatrix
        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
        multi = 1 / (2 * math.sin(theta))

        rx = multi * (R[2, 1] - R[1, 2]) * theta
        ry = multi * (R[0, 2] - R[2, 0]) * theta
        rz = multi * (R[1, 0] - R[0, 1]) * theta
        return rx, ry, rz

    def home_reset_position(self):
        home_position = [self.home_x, self.home_y, self.home_z]
        home_orientation = [90, 0, 0] # degrees
        rx, ry, rz = self.RPYtoRotVec(home_orientation)
        home_pose = home_position + [rx, ry, rz]
        self.robot.movel(home_pose, acc=self.acc, vel=self.vel)

    def test_point_inside(self, x, z):
        ellipse_eq = ((x - self.h) ** 2) / (self.a ** 2) + ((z - self.k) ** 2) / (self.b ** 2)
        if ellipse_eq <= 1:
            pass
        else:
            try:
                raise Exception("Point is outside the boundaries of the ellipse")
            except Exception as e:
                logging.info("out of boundaries", e)


    def working_angle_wrist(self):
        random_angle_pitch = random.uniform(self.min_angle_deg, self.max_angle_deg)
        orientation = [90, random_angle_pitch, 0]  # degrees
        rx, ry, rz = self.RPYtoRotVec(orientation)
        return rx, ry, rz

    def working_area_x_z(self):
        theta = random.uniform(0, 2 * math.pi)  # Generate random angle in radians
        radio = math.sqrt(random.uniform(0, 1))
        x = self.h + self.a * radio * math.cos(theta)
        z = self.k + self.b * radio * math.sin(theta)
        y = self.home_y  # keep the Y axis fix
        self.test_point_inside(x, z)
        return x, y, z


    def tool_move_pose_test(self):
        # move the tool in x, z planes and orientation around RY
        desire_point = self.working_area_x_z()  # (x, y, z) w.r.t to the base
        desire_orientation = self.working_angle_wrist()  # Angle Rotate around Ry
        desire_tool_pose =  desire_point + desire_orientation
        self.robot.movel(desire_tool_pose, acc=self.acc, vel=self.vel)



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
    robot_env.initial_position() # just making sure the joint are in the right position for initialization
    robot_env.home_reset_position()

    for i in range(10):
        robot_env.tool_move_pose_test()
    robot_env.home_reset_position()

    robot.close()


if __name__ == "__main__":
    main()
