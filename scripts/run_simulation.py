import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pybullet as p
import pybullet_data
import os
import time
import json
import numpy as np
from sim.world import World
from sim.robot import Robot
from sim.sensors.camera import Camera
from sim.sensors.imu import IMU


DATA_DIR = os.path.join(os.path.dirname(__file__), "../data")
RGB_DIR = os.path.join(DATA_DIR, "rgb")
DEPTH_DIR = os.path.join(DATA_DIR, "depth")
IMU_DIR = os.path.join(DATA_DIR, "imu")
os.makedirs(RGB_DIR, exist_ok=True)
os.makedirs(DEPTH_DIR, exist_ok=True)
os.makedirs(IMU_DIR, exist_ok=True)

metadata = {"num_samples": 0}

def main():
    physics_client = p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load world and robot
    world = World()
    world.load_plane()
    robot = Robot()
    robot.load()

    # Sensors
    camera = Camera(robot.robot_id)
    imu = IMU(robot.robot_id)

    # Simulation loop
    for step in range(100):
        robot.step()
        p.stepSimulation()
        time.sleep(1./240.)

        # Capture sensor data
        rgb, depth = camera.capture()
        imu_data = imu.read()

        # Save data
        np.save(os.path.join(RGB_DIR, f"{step:04d}.npy"), rgb)
        np.save(os.path.join(DEPTH_DIR, f"{step:04d}.npy"), depth)
        np.save(os.path.join(IMU_DIR, f"{step:04d}.npy"), imu_data)

        metadata["num_samples"] += 1

    # Save metadata
    with open(os.path.join(DATA_DIR, "metadata.json"), "w") as f:
        json.dump(metadata, f, indent=4)

    p.disconnect()

if __name__ == "__main__":
    main()
