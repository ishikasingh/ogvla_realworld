from arm_hand_deployment.consts import CONFIG_PATH
import hydra
from omegaconf import DictConfig, OmegaConf
from loguru import logger
import numpy as np
import time

from arm_hand_deployment.utils.grav_vec_trans import transform_gravity_vector
from arm_hand_deployment.utils.client_context import robot_client_context
from arm_hand_deployment.franka_allegro.communication.client import FrankaAllegroClient
from arm_hand_deployment.consts import FRANKA_EE_TO_ALLEGRO

# Define small variations around the default FRANKA_EE_TO_ALLEGRO rotation
TWIST = np.pi / 12
ROTATION_TARGETS = [
    [FRANKA_EE_TO_ALLEGRO[0], FRANKA_EE_TO_ALLEGRO[1], FRANKA_EE_TO_ALLEGRO[2]],  # Baseline (default)
    [FRANKA_EE_TO_ALLEGRO[0] + TWIST, FRANKA_EE_TO_ALLEGRO[1], FRANKA_EE_TO_ALLEGRO[2]],  # Slight roll
    [FRANKA_EE_TO_ALLEGRO[0], FRANKA_EE_TO_ALLEGRO[1] + TWIST, FRANKA_EE_TO_ALLEGRO[2]],  # Slight pitch
    [FRANKA_EE_TO_ALLEGRO[0], FRANKA_EE_TO_ALLEGRO[1], FRANKA_EE_TO_ALLEGRO[2] + TWIST],  # Slight yaw
    [FRANKA_EE_TO_ALLEGRO[0] + TWIST, FRANKA_EE_TO_ALLEGRO[1] + TWIST, FRANKA_EE_TO_ALLEGRO[2]],  # Tilted diagonally
    [FRANKA_EE_TO_ALLEGRO[0] - TWIST, FRANKA_EE_TO_ALLEGRO[1] - TWIST, FRANKA_EE_TO_ALLEGRO[2] + np.pi / 16],  # Another variation
]


@hydra.main(config_path=CONFIG_PATH, config_name="config", version_base="1.3")
def main(cfg: DictConfig):
    print(OmegaConf.to_yaml(cfg))

    port: int = int(cfg.rpc_port)
    server_ip: str = cfg.server_ip

    logger.info(f"Connecting to server {server_ip}:{port}")

    with robot_client_context(server_ip, port, FrankaAllegroClient) as client:
        client: FrankaAllegroClient

        # Get current pose
        ee_pose = client.GetEndEffectorPose()
        print(f"Initial End Effector Pose: {ee_pose}")

        for i, rpy in enumerate(ROTATION_TARGETS):
            logger.info(f"Moving to rotation {i + 1}: RPY = {rpy}")

            # Use the same XYZ position but apply new rotation
            target_end_effector_pose = [0.5, 0.1, 0.3] + rpy

            # Move the arm to the new orientation
            assert client.ArmMoveToEndEffectorPose(target_end_effector_pose)

            # Get updated pose
            ee_pose = client.GetEndEffectorPose()
            print(f"New EE Pose: {ee_pose}")

            # Compute and set the new gravity vector
            g_hand = transform_gravity_vector(ee_pose)
            client.SetGravityVector(g_hand)
            print(f"Updated Gravity Vector: {g_hand}")

            # Move the hand first
            if i == 0:
                target_hand_joint_positions = [0., 0.7, 0., 0.4, 0., 0.7, 0., 0.4, 0., 0.7, 0., 0.4, 1.,
                                        0., 0., 0.]
                assert client.HandMoveToJointPositions(target_hand_joint_positions)

            # Wait a bit before the next move
            time.sleep(1)

        logger.info("Rotation sequence completed.")

if __name__ == '__main__':
    main()
