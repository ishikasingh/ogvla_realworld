from arm_hand_deployment.consts import CONFIG_PATH

import hydra
from omegaconf import DictConfig, OmegaConf

from loguru import logger

from time import sleep, time

import numpy as np

import os

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

    BASE_DIR = '/home/yiyang/hsc/data/real_logs'

    object_0_name = '014_lemon'
    object_1_name = 'omniobject3d-can_001'

    run_id = 4

    state_dir = os.path.join(
        BASE_DIR, f"{object_0_name}_{object_1_name}", f'run_{run_id}')

    # Load initial joint positions
    initial_joint_positions_path = os.path.join(
        state_dir, 'initial_joint_positions.npy')

    initial_joint_positions_dict = np.load(
        initial_joint_positions_path, allow_pickle=True).item()

    initial_hand_joint_positions: np.ndarray = initial_joint_positions_dict[
        'hand_joint_positions']
    initial_arm_joint_positions: np.ndarray = initial_joint_positions_dict[
        'arm_joint_positions']

    # Load actions
    actions_path = '/home/yiyang/hsc/data/debug/actions.npy'
    actions = np.load(actions_path)

    port: int = int(cfg.rpc_port)
    server_ip: str = cfg.server_ip

    logger.info(f"Connecting to server {server_ip}:{port}")

    with robot_client_context(server_ip, port, FrankaAllegroClient) as client:
        client: FrankaAllegroClient

        joint_positions = np.concatenate(
            [initial_arm_joint_positions, initial_hand_joint_positions]
        )

        client.MoveToJointPositions(joint_positions=joint_positions)

        @timing_decorator
        def execute(action):
            assert client.ControlJointPositions(action=action)

        # warmup
        execute(joint_positions)

        print('-'*80)

        for action in actions:
            action = action.tolist()
            execute(action)


if __name__ == '__main__':
    main()
