from arm_hand_deployment.consts import CONFIG_PATH

import hydra
from omegaconf import DictConfig, OmegaConf

from loguru import logger

from time import sleep, time

import numpy as np

from arm_hand_deployment.utils.np_3d_utils import rpy_to_quaternion, quaternion_to_rpy, quaternion_multiply
from arm_hand_deployment.utils.grav_vec_trans import transform_gravity_vector
from arm_hand_deployment.utils.client_context import robot_client_context

from arm_hand_deployment.franka_allegro.communication.client import FrankaAllegroClient
from arm_hand_deployment.consts import FRANKA_EE_TO_ALLEGRO

@hydra.main(config_path=CONFIG_PATH, config_name="config", version_base="1.3")
def main(cfg: DictConfig):
    print(OmegaConf.to_yaml(cfg))

    port: int = int(cfg.rpc_port)
    server_ip: str = cfg.server_ip

    logger.info(f"Connecting to server {server_ip}:{port}")

    with robot_client_context(server_ip, port, FrankaAllegroClient) as client:
        client: FrankaAllegroClient

        # 7 joint positions for the arm + 1 for the gripper (width of the gripper)
        positions = client.GetJointPositions()

        # xyz+rpy
        ee_pose = client.GetEndEffectorPose()
        print(f"Joint positions: {positions}")
        print(f"End effector pose: {ee_pose}")

        target_joint_positions = [
            0.09162008114028396,
            -0.19826458111314524,
            -0.01990020486871322,
            -2.4732269941140346,
            -0.01307073642274261,
            2.30396583422025,
            0.8480939705504309,
        ]

        # 7 joint positions for the arm
        assert client.ArmMoveToJointPositions(target_joint_positions)

        positions = client.GetJointPositions()
        ee_pose = client.GetEndEffectorPose()
        print(f"Joint positions: {positions}")
        print(f"End effector pose: {ee_pose}")

        # You may need to change the last value here if you reassembled the hand...
        target_end_effector_pose = [0.5, 0.1, 0.3, FRANKA_EE_TO_ALLEGRO[0], FRANKA_EE_TO_ALLEGRO[1], FRANKA_EE_TO_ALLEGRO[2]]

        # xyz+rpy
        assert client.ArmMoveToEndEffectorPose(target_end_effector_pose)

        positions = client.GetJointPositions()
        ee_pose = client.GetEndEffectorPose()
        print(f"Joint positions: {positions}")
        print(f"End effector pose: {ee_pose}")

        g_hand = transform_gravity_vector(ee_pose)
        client.SetGravityVector(g_hand)

        target_hand_joint_positions = [0., 0.7, 0., 0.4, 0., 0.7, 0., 0.4, 0., 0.7, 0., 0.4, 1.,
                                       0., 0., 0.]

        assert client.HandMoveToJointPositions(target_hand_joint_positions)

        positions = client.GetJointPositions()
        ee_pose = client.GetEndEffectorPose()
        print(f"Joint positions: {positions}")
        print(f"End effector pose: {ee_pose}")


if __name__ == '__main__':
    main()
