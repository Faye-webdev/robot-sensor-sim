import pybullet as p
import numpy as np

class IMU:
    def __init__(self, robot_id):
        self.robot_id = robot_id

    def read(self):
        linear, angular = p.getBaseVelocity(self.robot_id)
        return np.array(list(linear) + list(angular))
