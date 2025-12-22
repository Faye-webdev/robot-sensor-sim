import pybullet as p
import numpy as np

class Camera:
    def __init__(self, robot_id):
        self.robot_id = robot_id

    def capture(self, width=64, height=64):
        pos = [0, 0, 1]
        target = [0, 0, 0]
        up = [0, 1, 0]
        view_matrix = p.computeViewMatrix(pos, target, up)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=width/height, nearVal=0.1, farVal=10)
        _, _, rgb, depth, _ = p.getCameraImage(width, height, view_matrix, proj_matrix)
        return np.array(rgb), np.array(depth)
