import pybullet as p

class World:
    def __init__(self):
        self.objects = []

    def load_plane(self):
        plane_id = p.loadURDF("plane.urdf")
        self.objects.append(plane_id)
        return plane_id