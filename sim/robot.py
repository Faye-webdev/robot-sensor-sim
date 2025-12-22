import pybullet as p
import math
import time

class Robot:
    def __init__(self, urdf_path="r2d2.urdf", start_pos=(0,0,0), start_ori=(0,0,0,1)):
        self.urdf_path = urdf_path
        self.start_pos = start_pos
        self.start_ori = start_ori
        self.robot_id = None
        self.num_joints = 0
        self.start_time = None  # track simulation start

    def load(self):
        import pybullet_data
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF(self.urdf_path, self.start_pos, self.start_ori)
        self.num_joints = p.getNumJoints(self.robot_id)
        self.start_time = time.time()
        return self.robot_id

    def move_joint(self, joint_index, target_position):
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_position
        )

    def step(self):
        if self.num_joints > 0:
            t = time.time() - self.start_time
            angle = 0.5 * math.sin(t)
            self.move_joint(0, angle)
