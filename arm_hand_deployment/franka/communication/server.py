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
                automatic_gripper_reset=False,
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

        # Define the controller type and configuration for the arm
        controller_type = "JOINT_IMPEDANCE"
        controller_cfg = RobotService._deoxys_joint_impedance_controller_cfg

        joint_positions = list(request.positions)
        assert len(joint_positions) == 7

        # Franka arm joints
        arm_target_joint_positions = np.array(joint_positions[:7])
        arm_initial_positions = np.array(RobotService._robot.last_q[:7])

        # Interpolation parameters
        num_steps = 100  # Number of interpolation steps
        if request.HasField("num_interpolation_steps"):
            num_steps = max(1, request.num_interpolation_steps)

        # Generate interpolation steps
        interpolated_positions = np.linspace(
            arm_initial_positions, arm_target_joint_positions, num_steps)

        for arm_interpolated_positions in interpolated_positions:
            arm_action = arm_interpolated_positions.tolist() + [-1.0]

            RobotService._robot.control(
                controller_type=controller_type,
                action=arm_action,
                controller_cfg=controller_cfg,
            )

        return service_pb2.Result(ok=service_pb2.Ok())

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
                    action = joint.tolist()
                    RobotService._robot.arm_control(
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
        action = np.concatenate((action_pos, action_ori))

        # Send action to the robot
        RobotService._robot.arm_control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg
        )

        return service_pb2.Result(ok=service_pb2.Ok())

    def ControlJointPositions(self, request, context):

        action = list(request.positions)

        assert len(action) == 7

        controller_type = "JOINT_IMPEDANCE"
        controller_cfg = RobotService._deoxys_joint_impedance_controller_cfg

        RobotService._robot.arm_control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg
        )

        return service_pb2.Result(ok=service_pb2.Ok())

    def GetGripperMessage(self, request, context):

        if RobotService._robot is None:
            context.set_details("Robot not started")
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            logger.error("GetGripperMessage: Robot not started")
            return service_pb2.GripperMessage()

        is_grasped = RobotService._robot.last_gripper_is_grasped

        return service_pb2.GripperMessage(is_grasped=is_grasped)

    def StopGripper(self, request, context):

        if RobotService._robot is None:
            context.set_details("Robot not started")
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            logger.error("StopGripper: Robot not started")
            return service_pb2.GripperMessage()

        RobotService._robot.stop_gripper()

        return service_pb2.Result(ok=service_pb2.Ok())
