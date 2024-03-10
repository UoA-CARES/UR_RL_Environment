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



class Env:
    def __init__(self, robot):
        self.robot = robot
        self.initial_set_up()
        self.vel = 1.0
        self.acc = 1.0

        self.home_x = 0.14
        self.home_y = -0.50
        self.home_z = 0.70

        self.Rx = math.radians(90)
        self.Ry = math.radians(0)
        self.Rz = math.radians(0)

        # wrist 3 range
        self.min_angle_deg = -10
        self.max_angle_deg = 10

        # ellipse area
        self.h, self.k = (self.home_x, self.home_z)  # central_point in (x, z)
        self.a = 0.40  # Semi-major axis length  x-axis
        self.b = 0.05  # Semi-minor axis length  z-axis



    def initial_set_up(self):
        self.robot.set_tcp((0, 0, 0, 0, 0, 0))  # Set tool central point
        self.robot.set_payload(0.5, (0, 0, 0))  # Kg

    def read_joint_state(self):
        joint_state = self.robot.getj()
        return joint_state

    def initial_position(self):
        home_pose = (1.5956498, -1.47229, 1.4730, -1.60006, 4.688987, 0.07035151) # Joint in rad for home position
        self.robot.movej(home_pose, vel=self.vel, acc=self.acc, wait=True)
        logging.info("Robot at initial position")

    def move_reset_position(self):
        home_position    = (self.home_x, self.home_y, self.home_z)
        home_orientation = (self.Rx, self.Ry, self.Rz)
        home_pose = home_position + home_orientation
        self.robot.movel(home_pose, vel=self.vel, acc=self.acc, wait=True)
        logging.info("Robot at reset pose")

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
        min_angle_rad = math.radians(self.min_angle_deg)
        max_angle_rad = math.radians(self.max_angle_deg)
        random_angle_rad = random.uniform(min_angle_rad, max_angle_rad)
        return random_angle_rad


    def working_area_x_z(self):
        theta = random.uniform(0, 2 * math.pi)  # Generate random angle in radians
        radio = math.sqrt(random.uniform(0, 1))

        x = self.h + self.a * radio * math.cos(theta)
        z = self.k + self.b * radio * math.sin(theta)
        y = self.home_y  # keep the Y axis fix
        self.test_point_inside(x, z)
        return x, y, z

    def tool_wrist_movement(self):
        # move the tool and rotating the orientation
        desire_point = self.working_area_x_z()  # (x, y, z) w.r.t to the base
        desire_angle = self.working_angle_wrist() # wrist3 angle Todo problmes here wrist out of limit

        t = self.robot.get_pose()  # Orientation and Vector, Transformation matrix from base to tcp
        t.orient.rotate_yb(desire_angle)  # rotate tcp around base y
        #t.pos[:3] = desire_point  # translate tcp in x,y,z  w.r.t base
        self.robot.set_pose(t, vel=self.vel, acc=self.acc, wait=True)  # move tcp to point and orientation defined by a transformation

    def tool_move_test(self):
        # move the tool in x, z planes, keeping the orientation fixed
        desire_point = self.working_area_x_z() # (x, y, z) w.r.t to the base
        desire_pose  = (self.Rx, self.Ry, self.Rz)
        desire_tool_pose =  desire_point + desire_pose
        self.robot.movel(desire_tool_pose, acc=self.acc, vel=self.vel)

    def tool_move_pose_test(self):
        # move the tool in x, z planes and orientation around RY
        desire_point = self.working_area_x_z()  # (x, y, z) w.r.t to the base

        #desire_angle = self.working_angle_wrist()  # Angle Rotate around Ry
        desire_angle = math.radians(-40)

        desire_pose =  (self.Rx, desire_angle, self.Rz)
        desire_tool_pose =  desire_point + desire_pose

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
    robot_env.initial_position() # just making sure the joint are in the right position
    robot_env.move_reset_position()


    for i in range(100000):
        logging.info(i)
        # robot_env.tool_move_test()
        robot_env.tool_move_pose_test()

    robot_env.move_reset_position()

    robot.close()


if __name__ == "__main__":
    main()
