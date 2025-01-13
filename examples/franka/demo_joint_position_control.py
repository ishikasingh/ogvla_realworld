from arm_hand_deployment.consts import CONFIG_PATH

import hydra
from omegaconf import DictConfig, OmegaConf

from loguru import logger

from time import sleep, time

import numpy as np

from arm_hand_deployment.utils.np_3d_utils import rpy_to_quaternion, quaternion_to_rpy, quaternion_multiply
from arm_hand_deployment.utils.client_context import robot_client_context

from arm_hand_deployment.franka.communication.client import FrankaClient


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

    port: int = int(cfg.rpc_port)
    server_ip: str = cfg.server_ip

    logger.info(f"Connecting to server {server_ip}:{port}")

    initial_joint_positions_dict = np.load(
        '/home/abrar/Downloads/2025-01-13-14-50-28/initial_joint_positions.npy', allow_pickle=True).item()

    initial_arm_joint_positions = initial_joint_positions_dict[
        'arm_joint_positions'].tolist()

    actions = np.load('/home/abrar/Downloads/2025-01-13-14-50-28/actions.npy')

    with robot_client_context(server_ip, port, FrankaClient) as client:
        client: FrankaClient

        assert client.MoveToJointPositions(initial_arm_joint_positions)

        @timing_decorator
        def execute(action):
            assert client.ControlJointPositions(action=action)

        # warmup
        execute(initial_arm_joint_positions)

        print('-'*80)

        for action in actions:
            action = action[:7].tolist()
            execute(action)


if __name__ == '__main__':
    main()
