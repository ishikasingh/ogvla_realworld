import os
import numpy as np

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "config")

FRANKA_EE_TO_ALLEGRO = [np.pi, 0, -np.pi/4]
