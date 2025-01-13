from arm_hand_deployment.franka.proto import service_pb2, service_pb2_grpc

import grpc

import numpy as np
from arm_hand_deployment.utils.np_3d_utils import (
    quaternion_to_rpy,
    rpy_to_quaternion,
    quaternion_multiply_translation,
    quaternion_inverse,
    rpy_to_rotation_matrix,
    quaternion_slerp)

from typing import Optional, Dict, Any, List

from loguru import logger

from time import sleep, time
import traceback

from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig, transform_utils
from deoxys.utils.ik_utils import IKWrapper


class RobotService(service_pb2_grpc.FrankaService):
    # Shared attribute across all instances of RobotService
    _robot: Optional[FrankaInterface] = None

    _fci_ip: Optional[str] = None
    _deoxys_general_cfg_file: Optional[str] = None
    _deoxys_osc_controller_cfg: Optional[Dict[str, Any]] = None
    _deoxys_joint_position_controller_cfg: Optional[Dict[str, Any]] = None
    _deoxys_joint_impedance_controller_cfg: Optional[Dict[str, Any]] = None

    def __init__(self, fci_ip: str,
                 deoxys_general_cfg_file: str,
                 deoxys_osc_controller_cfg_file: str,
                 deoxys_joint_position_controller_cfg_file: str,
                 deoxys_joint_impedance_controller_cfg_file: str,
                 ):

        # Only set _fci_ip if it's not already set (persistent across requests)
        if RobotService._fci_ip is None:
            RobotService._fci_ip = fci_ip

        if RobotService._deoxys_general_cfg_file is None:
            RobotService._deoxys_general_cfg_file = deoxys_general_cfg_file

        if RobotService._deoxys_osc_controller_cfg is None:
            RobotService._deoxys_osc_controller_cfg = YamlConfig(
                deoxys_osc_controller_cfg_file).as_easydict()

        if RobotService._deoxys_joint_position_controller_cfg is None:
            RobotService._deoxys_joint_position_controller_cfg = YamlConfig(
                deoxys_joint_position_controller_cfg_file).as_easydict()

        if RobotService._deoxys_joint_impedance_controller_cfg is None:
            RobotService._deoxys_joint_impedance_controller_cfg = YamlConfig(
                deoxys_joint_impedance_controller_cfg_file).as_easydict()

    # By design, we didn't launch the actual franka control process here.

    def Start(self, request, context):

        if RobotService._robot is None:
            RobotService._robot = FrankaInterface(
                general_cfg_file=RobotService._deoxys_general_cfg_file,
                has_gripper=True,
            )

            sleep(1)
        return service_pb2.Result(ok=service_pb2.Ok())

    def Stop(self, request, context):
        if RobotService._robot is None:
            logger.error("Robot not started")
            return service_pb2.Result(err=service_pb2.Err(message="Robot not started"))

        logger.warning(
            "In fact, I didn't stop anything.")

        return service_pb2.Result(ok=service_pb2.Ok())

    def GetJointPositions(self, request, context):
        if RobotService._robot is None:
            context.set_details("Robot not started")
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            logger.error("GetJointPositions: Robot not started")
            return service_pb2.JointPositions()

        arm_q = RobotService._robot.last_q

        hand_q = RobotService._robot.last_gripper_q

        positions = np.zeros(8)
        positions[:7] = arm_q
        positions[7] = hand_q

        return service_pb2.JointPositions(positions=positions)

    def GetEndEffectorPose(self, request, context):
        if RobotService._robot is None:
            context.set_details("Robot not started")
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            logger.error("GetEndEffectorPose: Robot not started")
            return service_pb2.Pose()

        ee_quat, ee_pos = RobotService._robot.last_eef_quat_and_pos
        ee_pos = ee_pos.squeeze(1)

        rpy = quaternion_to_rpy(ee_quat)

        return service_pb2.Pose(pose=np.concatenate((ee_pos, rpy)))

    def MoveToJointPositions(self, request, context):
        if RobotService._robot is None:
            logger.error("Robot not started")
            return service_pb2.Result(err=service_pb2.Err(message="Robot not started"))

        try:
            # Define the controller type and configuration for the arm
            controller_type = "JOINT_POSITION"
            controller_cfg = RobotService._deoxys_joint_position_controller_cfg

            joint_positions = list(request.positions)

            assert len(joint_positions) == 7

            arm_action = joint_positions + [-1.0]

            # Maximum number of attempts to reach the target positions
            max_attempts = 300
            cnt = 0

            threshold_arm = 1e-3

            # Control loop to move both the arm and the hand to their target positions
            while True:
                # Check the current joint positions of the arm
                arm_current_positions = RobotService._robot.last_q

                # Break the loop if the joints are within a small tolerance of the target for both arm and hand
                delta_arm = np.max(
                    np.abs(np.array(arm_current_positions) - np.array(joint_positions)))
                arm_reached = delta_arm < threshold_arm

                if arm_reached:
                    break

                if cnt >= max_attempts:
                    warning_message = "Failed to reach target joint positions within the maximum attempts."
                    logger.warning(warning_message)
                    return service_pb2.Result(err=service_pb2.Err(message=warning_message))

                RobotService._robot.control(
                    controller_type=controller_type,
                    action=arm_action,
                    controller_cfg=controller_cfg,
                )

                # Optional: Add a small sleep to prevent excessive command frequency
                sleep(0.01)
                cnt += 1

            return service_pb2.Result(ok=service_pb2.Ok())

        except Exception as e:
            context.set_details(f"Failed to move to joint positions: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            logger.error(f"MoveToJointPositions error: {e}")
            return service_pb2.Result(err=service_pb2.Err(message="Failed to move to joint positions"))

    def MoveToEndEffectorPose(self, request, context):
        if RobotService._robot is None:
            logger.error("Robot not started")
            return service_pb2.Result(err=service_pb2.Err(message="Robot not started"))

        try:
            # Use Joint Impedance controller configuration for arm
            controller_type = "JOINT_IMPEDANCE"
            # Use the impedance controller config
            controller_cfg = RobotService._deoxys_joint_impedance_controller_cfg

            # Initialize IKWrapper for inverse kinematics calculation
            ik_wrapper = IKWrapper()

            # Extract target pose (xyz + rpy) and convert rpy to rotation matrix
            target_position: np.ndarray = np.array(request.pose[:3])

            target_rpy = request.pose[3:]
            target_mat = rpy_to_rotation_matrix(target_rpy)

            # Get the current joint positions
            current_joint_positions = RobotService._robot.last_q

            # Compute IK to get joint trajectory to the target end-effector pose
            joint_traj, debug_info = ik_wrapper.ik_trajectory_to_target_pose(
                target_position, target_mat, current_joint_positions, num_points=20
            )

            if joint_traj is None:
                error_message = "IK failed to compute a valid trajectory."
                logger.error(error_message)
                return service_pb2.Result(err=service_pb2.Err(message=error_message))

            # Interpolate the trajectory for smooth motion
            joint_traj = ik_wrapper.interpolate_dense_traj(joint_traj)

            # Function to execute the arm movement using the IK result
            def execute_ik_result():
                for joint in joint_traj:
                    # Append -1.0 to indicate the end of the command
                    action = joint.tolist() + [-1.0]
                    RobotService._robot.control(
                        controller_type=controller_type,
                        action=action,
                        controller_cfg=controller_cfg,
                    )

            # Execute the arm movement
            execute_ik_result()

            logger.info("MoveToEndEffectorPose executed successfully")
            return service_pb2.Result(ok=service_pb2.Ok())

        except Exception as e:
            context.set_details(
                f"Failed to move to target arm end-effector pose: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            logger.error(traceback.format_exc())
            logger.error(f"MoveToEndEffectorPose error: {e}")
            return service_pb2.Result(err=service_pb2.Err(message="Failed to execute MoveToEndEffectorPose"))

    def SetGripperAction(self, request, context):
        action = request.value
        RobotService._robot.gripper_control(action)
        return service_pb2.Result(ok=service_pb2.Ok())

    def ControllDeltaEndEffectorPose(self, request, context):
        delta_pose = request.delta_pose

        assert len(delta_pose) == 6

        controller_type = "OSC_POSE"
        controller_cfg = RobotService._deoxys_osc_controller_cfg

        # Compute delta translation and rotation from the request

        delta_rpy = np.array(delta_pose[3:])
        delta_quat = rpy_to_quaternion(delta_rpy)

        action_pos = np.array(delta_pose[:3])
        action_ori = transform_utils.quat2axisangle(delta_quat)

        # Construct the action vector
        action = np.concatenate((action_pos, action_ori, [-1.0]))

        # Send action to the robot
        RobotService._robot.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg
        )

        return service_pb2.Result(ok=service_pb2.Ok())

    def ControlJointPositions(self, request, context):
        action = request.positions

        assert len(action) == 7

        controller_type = "JOINT_POSITION"
        controller_cfg = RobotService._deoxys_joint_position_controller_cfg

        action = action + [-1.0]

        RobotService._robot.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg
        )

        return service_pb2.Result(ok=service_pb2.Ok())
