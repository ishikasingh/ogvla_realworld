#!/usr/bin/env python3
"""
sine_wave_controller.py

This script connects to a robot and executes a sine-wave based trajectory.
A reusable SinTrajectory class is provided to compute joint positions according to:
    position(t) = initial_position + amplitude * sin(2π * frequency * t + phase)

This version supports testing with:
    - 23 DoF mode (arm joint positions + hand joint positions)
    - 22 DoF mode (arm end-effector pose + hand joint positions)

Modify the `MODE` variable to switch between them.
"""

from arm_hand_deployment.consts import CONFIG_PATH

import hydra
from omegaconf import DictConfig, OmegaConf

from loguru import logger

from time import sleep, time
import numpy as np

from arm_hand_deployment.utils.grav_vec_trans import transform_gravity_vector
from arm_hand_deployment.utils.client_context import robot_client_context
from arm_hand_deployment.franka_allegro.communication.client import FrankaAllegroClient


class SinTrajectory:
    """
    Generate a sine wave trajectory for any robot with multiple DoFs.

    The trajectory is computed as:
        position(t) = initial_position + amplitude * sin(2π * frequency * t + phase)

    Args:
        initial_position (np.ndarray): The initial joint positions.
        amplitude (np.ndarray): The amplitude for the sine wave for each DoF.
        frequency (np.ndarray): The frequency for the sine wave for each DoF.
        phase (np.ndarray, optional): Phase offset for each DoF (defaults to zero).
    """
    def __init__(
        self,
        initial_position: np.ndarray,
        amplitude: np.ndarray,
        frequency: np.ndarray,
        phase: np.ndarray = None,
    ):
        self.initial_position = initial_position
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase = phase if phase is not None else np.zeros_like(initial_position)

    def get_position_at_time(self, t: float) -> np.ndarray:
        """
        Compute the joint positions at time t.

        Args:
            t (float): Time (in seconds) at which to compute the trajectory.

        Returns:
            np.ndarray: The joint positions at time t.
        """
        return self.initial_position + self.amplitude * np.sin(2 * np.pi * self.frequency * t + self.phase)


# === MODE CONFIGURATION ===
# "23DOF": Use arm joint positions + hand joint positions (23 DoF total)
# "22DOF": Use arm end-effector pose + hand joint positions (22 DoF total)
MODE = "22DOF"  # Change this to "22DOF" if needed.

# === Define Initial Joint Positions Inside Script ===
# Modify these values according to your robot's setup.
DEFAULT_INITIAL_JOINT_POSITIONS = {
    "arm_ee_pose": [  # 6 DoFs
        0.5, 0.0, 0.3, np.pi, 0, - np.pi / 4
    ],
    "arm_joint_positions": [  # 7 DoFs
        2.7071e-01, -4.7451e-02, -2.5846e-01, -2.4447e+00, -1.1587e-02, 2.1987e+00,  1.5921e+00
    ],
    "hand_joint_positions": [  # 16 DoFs
        0, 0.3, 0.4, 1.2,  # ring
        0, 0.3, 0.4, 1.2,  # middle
        0, 0.3, 0.4, 1.2,   # index
        0.8, 0.3, 0.4, 1.2,  # thumb
    ]
}
# Notes: for rot (3 DoF), see from the back of the hand to the front direction (wrist POV)
# All center: []
# first => 左歪头，右歪头
# second => 上抬头，下抬头
# third => 左转向，右转向

# === Sine Trajectory Configuration ===
if MODE == "23DOF":
    num_dofs = 23  # 7 arm joint positions + 16 hand joint positions
    joint_positions = np.concatenate([
        DEFAULT_INITIAL_JOINT_POSITIONS["arm_joint_positions"],
        DEFAULT_INITIAL_JOINT_POSITIONS["hand_joint_positions"]
    ])
elif MODE == "22DOF":
    num_dofs = 22  # 6 arm end-effector pose + 16 hand joint positions
    joint_positions = np.concatenate([
        DEFAULT_INITIAL_JOINT_POSITIONS["arm_ee_pose"],
        DEFAULT_INITIAL_JOINT_POSITIONS["hand_joint_positions"]
    ])
else:
    raise ValueError("Invalid mode. Choose either '23DOF' or '22DOF'.")

# Initialize all parameters as zero
DEFAULT_SIN_CONFIG = {
    "amplitude": np.zeros(num_dofs),
    "frequency": np.zeros(num_dofs),
    "phase": np.zeros(num_dofs),
    "duration": 10.0,   # Total duration of the trajectory (seconds)
    "update_rate": 20 # Control update rate (Hz)
}

# === Define Sine Motion on Specific Joints ===
arm_dof = 7 if MODE == "23DOF" else 6  # Number of DoFs in the arm part

index_indices = np.array([arm_dof +1, arm_dof + 5, arm_dof + 9])  # Target joints
ee_pose_indices = np.array([3,])  # Target joints

sine_indices = index_indices
sine_amplitude = 0.5
sine_frequency = 1.0    

# Apply sine wave settings only to the selected joints
DEFAULT_SIN_CONFIG["amplitude"][sine_indices] = sine_amplitude
DEFAULT_SIN_CONFIG["frequency"][sine_indices] = sine_frequency


@hydra.main(config_path=CONFIG_PATH, config_name="config", version_base="1.3")
def main(cfg: DictConfig):
    # Print out the loaded configuration (server_ip, rpc_port)
    print(OmegaConf.to_yaml(cfg))

    # === Set Up Sine Wave Trajectory ===
    amplitude = DEFAULT_SIN_CONFIG["amplitude"]
    frequency = DEFAULT_SIN_CONFIG["frequency"]
    phase = DEFAULT_SIN_CONFIG["phase"]
    duration = DEFAULT_SIN_CONFIG["duration"]
    update_rate = DEFAULT_SIN_CONFIG["update_rate"]

    # Create an instance of the sine trajectory generator.
    sin_traj = SinTrajectory(
        initial_position=joint_positions,
        amplitude=amplitude,
        frequency=frequency,
        phase=phase,
    )

    # === Connect to Robot ===
    port: int = int(cfg.rpc_port)
    server_ip: str = cfg.server_ip

    logger.info(f"Connecting to server {server_ip}:{port}")

    with robot_client_context(server_ip, port, FrankaAllegroClient) as client:
        client: FrankaAllegroClient

        # Move the robot to the initial joint positions.
        if MODE == "23DOF":
            client.MoveToJointPositions(joint_positions=joint_positions)
            # client.ArmMoveToJointPositions(positions=joint_positions[:arm_dof])
            # client.HandMoveToJointPositions(positions=joint_positions[arm_dof:])
        elif MODE == "22DOF":
            client.ArmMoveToEndEffectorPose(pose=joint_positions[:arm_dof])
            client.HandMoveToJointPositions(positions=joint_positions[arm_dof:])

        logger.info("Starting sine wave trajectory execution.")
        start_time = time()
        dt = 1.0 / update_rate  # Time step between commands

        # Execute the sine trajectory for the given duration.
        current_time = 0.0
        while current_time < duration:
            t = current_time
            new_positions = sin_traj.get_position_at_time(t)

            ee_pose = client.GetEndEffectorPose()
            g_hand = transform_gravity_vector(ee_pose)
            client.SetGravityVector(g_hand)

            # Send the new joint positions to the robot.
            if MODE == "23DOF":
                client.ControlJointPositions(action=new_positions.tolist())
            elif MODE == "22DOF":
                client.ControlArmEEPoseHandJointPositions(action=new_positions.tolist())
            sleep(dt)
            current_time = time() - start_time

        logger.info("Completed sine wave trajectory execution.")


if __name__ == "__main__":
    main()
