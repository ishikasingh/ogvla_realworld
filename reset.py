from arm_hand_deployment.consts import CONFIG_PATH

import hydra
from omegaconf import DictConfig, OmegaConf

from loguru import logger

from time import sleep, time

import numpy as np

from arm_hand_deployment.utils.np_3d_utils import rpy_to_quaternion, quaternion_to_rpy, quaternion_multiply
from arm_hand_deployment.utils.client_context import robot_client_context

from arm_hand_deployment.franka_allegro.communication.client import FrankaAllegroClient


def timing_decorator(func):
    def wrapper(*args, **kwargs):
        start_time = time()  # Record start time
        result = func(*args, **kwargs)  # Call the function
        end_time = time()  # Record end time
        execution_time = end_time - start_time  # Calculate execution time
        print(
            f"Execution time of {func.__name__}: {execution_time:.4f} seconds")
        return result

    return wrapper


@hydra.main(config_path=CONFIG_PATH, config_name="config", version_base="1.3")
def main(cfg: DictConfig):
    print(OmegaConf.to_yaml(cfg))

    initial_hand_joint_positions: np.ndarray = np.array(
        [0., 0.7, 0., 0.4, 0., 0.7, 0., 0.4, 0., 0.7, 0., 0.4, 1.,
         0., 0., 0.]
    )
    initial_arm_joint_positions: np.ndarray = np.array(
        [-0.14377305, -0.41615936, 0.15588209, -2.7992177, 0.09156579,
         2.3855371, 1.5030054]
    )

    port: int = int(cfg.rpc_port)
    server_ip: str = cfg.server_ip

    logger.info(f"Connecting to server {server_ip}:{port}")

    with robot_client_context(server_ip, port, FrankaAllegroClient) as client:
        client: FrankaAllegroClient

        joint_positions = np.concatenate(
            [initial_arm_joint_positions, initial_hand_joint_positions]
        )

        client.MoveToJointPositions(joint_positions=joint_positions)


if __name__ == '__main__':
    main()
