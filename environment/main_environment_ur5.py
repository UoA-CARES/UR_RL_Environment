import math
import random
import logging
logging.basicConfig(level=logging.INFO)
import collections
collections.Iterable = collections.abc.Iterable # Need this for math3d lib issues

from tools.util import prepare_point


class Environment:
    def __init__(self, robot, velocity=0.1, acceleration=0.5):
        self.robot = robot

        self.robot.set_tcp((0, 0, 0, 0, 0, 0))  # Set tool central point
        self.robot.set_payload(0.4, (0, 0, 0))  # Kg

        self.vel = velocity
        self.acc = acceleration

        # Home Position
        self.home_position    = (0.14, -0.50, 0.40) # X, Y , Z
        self.home_orientation = (90.0, 0.0, 0.0)  # Roll, Pith , Yaw  in Degrees

        # Working Area
        self.h, self.k = (self.home_position[0], self.home_position[2])  # central_point in (x, z)
        self.a = 0.30  # Semi-major axis length  x-axis
        self.b = 0.25  # Semi-minor axis length  z-axis

        # wrist 3 range
        self.min_angle_rotation = -40
        self.max_angle_rotation = 40


    def test_position(self, x, y, z):
        ellipse_eq = ((x - self.h) ** 2) / (self.a ** 2) + ((z - self.k) ** 2) / (self.b ** 2)
        if ellipse_eq > 1:
            logging.error("Point X o Z is outside the boundaries of the ellipse")
            return False

        y_tolerance = 0.001
        y_min = self.home_position[1] - y_tolerance
        y_max = self.home_position[1] + y_tolerance
        if not (y_min <= y <= y_max):
            logging.error(f"Y axis is outside the boundary, should be fixed to {self.home_position[1]}")
            return False
        return True

    def test_orientation(self, r, p, y):
        if r != self.home_orientation[0] or p < self.min_angle_rotation or p > self.max_angle_rotation or y != self.home_orientation[2]:
            logging.error("Angle is outside the boundaries for rotation")
            return False
        return True

    def check_point(self, pose):
        valid_position = self.test_position(pose[0], pose[1], pose[2])
        valid_orientation = self.test_orientation(pose[3], pose[4], pose[5])
        if not (valid_position and valid_orientation):
            logging.error("Not valid pose, sending robot to home position")
            move_pose = prepare_point((self.home_position + self.home_orientation))
            return move_pose
        return prepare_point(pose)


    def sample_position(self):
        theta = random.uniform(0, 2 * math.pi)  # Generate random angle in radians
        radio = math.sqrt(random.uniform(0, 1))
        x = self.h + self.a * radio * math.cos(theta)
        z = self.k + self.b * radio * math.sin(theta)
        y = self.home_position[1]  # keep the Y axis fix
        return x, y, z

    def sample_orientation(self):
        roll  = self.home_orientation[0]
        pitch = random.uniform(self.min_angle_rotation, self.max_angle_rotation)
        yaw = self.home_orientation[2]
        return roll, pitch, yaw

    def starting_position(self):
        initial_position = (math.radians(90), math.radians(-90), math.radians(90), math.radians(0), math.radians(90), math.radians(0))  # Joint in rad for home position
        self.robot.movej(initial_position, vel=self.vel, acc=self.acc, wait=True)
        logging.info("Robot at initial position")

    def robot_home_position(self):
        desire_pose = prepare_point((self.home_position + self.home_orientation))
        self.robot.movel(desire_pose, acc=self.acc, vel=self.vel)
        logging.info("Robot at home position")

    def get_sample_pose(self):
        desire_position = self.sample_position()  # (x, y, z) w.r.t to the base
        desire_orientation = self.sample_orientation() # r p y)
        desire_pose = self.check_point((desire_position + desire_orientation))
        return desire_pose

    def tool_move_pose_test(self):
        desire_tool_pose   =  self.get_sample_pose()
        self.robot.movel(desire_tool_pose, acc=self.acc, vel=self.vel)
        logging.info("Move completed")

    def hard_code_solution(self):

        for _ in range(5):

            # pose 1
            desire_position_1    = (self.home_position[0]-0.15, self.home_position[1], self.home_position[2])
            desire_orientation_1 = (self.home_orientation[0], self.home_orientation[1]+30, self.home_orientation[2])
            desire_pose_1 = self.check_point((desire_position_1+desire_orientation_1))

            # pose 3
            desire_position_3 = (self.home_position[0], self.home_position[1], self.home_position[2] - 0.02)
            desire_orientation_3 = (self.home_orientation[0], self.home_orientation[1], self.home_orientation[2])
            desire_pose_3 = self.check_point((desire_position_3 + desire_orientation_3))

            # pose 2
            desire_position_2    = (self.home_position[0]+0.15, self.home_position[1], self.home_position[2])
            desire_orientation_2 = (self.home_orientation[0], self.home_orientation[1]-30, self.home_orientation[2])
            desire_pose_2 = self.check_point((desire_position_2+desire_orientation_2))

            self.robot.movels([desire_pose_1, desire_pose_3, desire_pose_2, desire_pose_3], vel=self.vel, acc=self.acc, radius=0.01)







    def get_state(self):
        pass
        # working here







