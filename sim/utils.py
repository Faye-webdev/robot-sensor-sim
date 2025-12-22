import numpy as np

def add_noise(sensor_value, noise_std=0.01):
    return sensor_value + np.random.normal(0, noise_std, size=np.shape(sensor_value))

def normalize(value, min_val=0, max_val=1):
    return (value - min_val) / (max_val - min_val)