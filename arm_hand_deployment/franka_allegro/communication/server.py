from arm_hand_deployment.franka_allegro.proto import service_pb2, service_pb2_grpc

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

from copy import deepcopy

from loguru import logger

from time import sleep, time
import traceback

from deoxys.franka_interface import FrankaInterface
from deoxys.utils import YamlConfig, transform_utils
from deoxys.utils.ik_utils import IKWrapper

# NOTE (hsc): This is a ROS package.
from allegro_hand.controller import AllegroController


class RobotService(service_pb2_grpc.FrankaAllegroService):
    # Shared attribute across all instances of RobotService
    _robot: Optional[FrankaInterface] = None

    _allegro_controller: Optional[AllegroController] = None

    _fci_ip: Optional[str] = None
    _deoxys_general_cfg_file: Optional[str] = None
    _deoxys_osc_controller_cfg: Optional[Dict[str, Any]] = None
    # absolute OSC
    _deoxys_osc_controller_cfg_no_delta: Optional[Dict[str, Any]] = None
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
            RobotService._deoxys_osc_controller_cfg_no_delta = deepcopy(
                RobotService._deoxys_osc_controller_cfg)
            RobotService._deoxys_osc_controller_cfg_no_delta.is_delta = False

        if RobotService._deoxys_joint_position_controller_cfg is None:
            RobotService._deoxys_joint_position_controller_cfg = YamlConfig(
                deoxys_joint_position_controller_cfg_file).as_easydict()

        if RobotService._deoxys_joint_impedance_controller_cfg is None:
            RobotService._deoxys_joint_impedance_controller_cfg = YamlConfig(
                deoxys_joint_impedance_controller_cfg_file).as_easydict()

        # Start the Allegro Hand controller
        # NOTE (hsc): If I initialize the ROS node in Start, there is something very weird.
        # I don't know why.
        if RobotService._allegro_controller is None:
            RobotService._allegro_controller = AllegroController()
        else:
            logger.warning("Allegro hand already started")

    # By design, we didn't launch the actual franka control process here.

    def Start(self, request, context):

        if RobotService._robot is None:
            RobotService._robot = FrankaInterface(
                general_cfg_file=RobotService._deoxys_general_cfg_file,
                has_gripper=False,
                automatic_gripper_reset=False,
            )

            sleep(1)

        initial_hand_joint_positions = RobotService._allegro_controller.current_joint_pose.position
        print("Initial hand joint positions:")
        print(initial_hand_joint_positions)
        RobotService._allegro_controller.hand_pose(
            initial_hand_joint_positions)

        sleep(1)

        return service_pb2.Result(ok=service_pb2.Ok())

    def Stop(self, request, context):
        if RobotService._robot is None:
            logger.error("Robot not started")
            return service_pb2.Result(err=service_pb2.Err(message="Robot not started"))

        logger.warning(
            "In fact, I didn't stop anything.")

        return service_pb2.Result(ok=service_pb2.Ok())

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
    
    def SetGravityVector(self, request, context):
        if RobotService._allegro_controller is None:
            logger.error("Allegro hand not started")
            return service_pb2.Result(err=service_pb2.Err(message="Allegro hand not started"))

        gravity_vector = list(request.vector)

        # Apply the new gravity vector
        RobotService._allegro_controller.apply_grav_comp(gravity_vector)

        logger.info(f"Updated Gravity Vector: {gravity_vector}")

        return service_pb2.Result(ok=service_pb2.Ok())

    def MoveToJointPositions(self, request, context):
        if RobotService._robot is None:
            logger.error("Robot not started")
            return service_pb2.Result(err=service_pb2.Err(message="Robot not started"))

        if RobotService._allegro_controller is None:
            logger.error("Allegro hand not started")
            return service_pb2.Result(err=service_pb2.Err(message="Allegro hand not started"))

        # Define the controller type and configuration for the arm
        controller_type = "JOINT_POSITION"
        controller_cfg = RobotService._deoxys_joint_position_controller_cfg

        joint_positions = list(request.positions)

        # Split joint positions for the arm and hand
        # Franka arm joints
        arm_target_joint_positions = joint_positions[:7]
        # Allegro hand joints (expected to be 16 values)
        hand_target_joint_positions = joint_positions[7:]

        # Append -1.0 to indicate the end of command for the arm
        arm_action = arm_target_joint_positions + [-1.0]

        # Maximum number of attempts to reach the target positions
        max_attempts = 300
        cnt = 0

        threshold_arm = 2e-3
        threshold_hand_pos = 0.1  # 5e-2

        # Control loop to move both the arm and the hand to their target positions
        while True:
            # Check the current joint positions of the arm
            arm_current_positions = RobotService._robot.last_q

            # Check the current joint positions of the hand
            hand_current_positions = np.array(
                RobotService._allegro_controller.current_joint_pose.position)

            # Break the loop if the joints are within a small tolerance of the target for both arm and hand
            delta_arm = np.max(
                np.abs(np.array(arm_current_positions) - np.array(arm_target_joint_positions)))
            arm_reached = delta_arm < threshold_arm

            # there seems to be some friction between joint 12 and the 3d printed link.

            delta_hand = np.abs(
                hand_current_positions - hand_target_joint_positions)
            print(
                f"[Joint 12] Current:{hand_current_positions[12]:.4f}\tTarget:{hand_target_joint_positions[12]:.4f}\tDelta:{delta_hand[12]:.4f}")
            delta_hand[12] = 0

            # print(delta_hand)
            delta_hand = np.max(delta_hand)
            hand_reached = delta_hand < threshold_hand_pos

            print(
                f"Max delta arm:{delta_arm:.4f}\tMax delta hand:{delta_hand:.4f}")

            if arm_reached and hand_reached and cnt > 20:
                break

            if cnt >= max_attempts:
                warning_message = "Failed to reach target joint positions within the maximum attempts."
                logger.warning(warning_message)
                return service_pb2.Result(err=service_pb2.Err(message=warning_message))

            # Send the action to the robot's control method for the arm
            if not arm_reached:
                RobotService._robot.control(
                    controller_type=controller_type,
                    action=arm_action,
                    controller_cfg=controller_cfg,
                )

            # Update the hand joint positions
            if not hand_reached:
                RobotService._allegro_controller.hand_pose(
                    hand_target_joint_positions)

            # Optional: Add a small sleep to prevent excessive command frequency
            sleep(0.01)
            cnt += 1

        return service_pb2.Result(ok=service_pb2.Ok())

    def GetJointPositions(self, request, context):
        if RobotService._robot is None:
            context.set_details("Robot not started")
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            logger.error("GetJointPositions: Robot not started")
            return service_pb2.JointPositions()

        if RobotService._allegro_controller is None:
            context.set_details("Allegro hand not started")
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            logger.error("GetJointPositions: Allegro hand not started")
            return service_pb2.JointPositions()

        arm_q = RobotService._robot.last_q

        hand_q = np.array(
            RobotService._allegro_controller.current_joint_pose.position)

        return service_pb2.JointPositions(positions=np.concatenate([arm_q, hand_q]))

    def ArmMoveToJointPositions(self, request, context):
        if RobotService._robot is None:
            logger.error("Robot not started")
            return service_pb2.Result(err=service_pb2.Err(message="Robot not started"))

        if RobotService._allegro_controller is None:
            logger.error("Allegro hand not started")
            return service_pb2.Result(err=service_pb2.Err(message="Allegro hand not started"))

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

    def ArmMoveToEndEffectorPose(self, request, context):
        if RobotService._robot is None:
            logger.error("Robot not started")
            return service_pb2.Result(err=service_pb2.Err(message="Robot not started"))

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
            target_position, target_mat, current_joint_positions, num_points=100
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

    def HandMoveToJointPositions(self, request, context):
        if RobotService._allegro_controller is None:
            context.set_details("Allegro hand not started")
            context.set_code(grpc.StatusCode.FAILED_PRECONDITION)
            logger.error("HandMoveToJointPositions: Allegro hand not started")
            return service_pb2.Result(err=service_pb2.Err(message="Robot not started"))

        hand_target_joint_positions = np.array(request.positions)

        assert len(hand_target_joint_positions) == 16

        max_attempts = 200
        cnt = 0

        threshold_hand_pos = 0.1  # 5e-2
        while True:

            # Check the current joint positions of the hand
            hand_current_positions = np.array(
                RobotService._allegro_controller.current_joint_pose.position)

            # NOTE (hsc): there seems to be some friction between joint 12 and the 3d printed link.

            delta_hand = np.abs(
                hand_current_positions - hand_target_joint_positions)
            print(
                f"[Joint 12] Current:{hand_current_positions[12]:.4f}\tTarget:{hand_target_joint_positions[12]:.4f}\tDelta:{delta_hand[12]:.4f}")
            delta_hand[12] = 0

            delta_hand = np.max(delta_hand)

            print(f"Max delta hand:{delta_hand:.4f}")

            hand_reached = delta_hand < threshold_hand_pos

            if hand_reached and cnt > 20:
                break

            if cnt >= max_attempts:
                warning_message = "Failed to reach target joint positions within the maximum attempts."
                logger.warning(warning_message)
                return service_pb2.Result(err=service_pb2.Err(message=warning_message))

            RobotService._allegro_controller.hand_pose(
                hand_target_joint_positions)

            sleep(0.01)
            cnt += 1

        return service_pb2.Result(ok=service_pb2.Ok())

    def ControlJointPositions(self, request, context):

        if RobotService._robot is None:
            logger.error("Robot not started")
            return service_pb2.Result(err=service_pb2.Err(message="Robot not started"))

        if RobotService._allegro_controller is None:
            logger.error("Allegro hand not started")
            return service_pb2.Result(err=service_pb2.Err(message="Allegro hand not started"))

        joint_positions = np.array(request.positions)

        arm_joint_positions = joint_positions[:7]
        hand_joint_positions = joint_positions[7:]

        # print(RobotService._allegro_controller.current_joint_pose.position)
        RobotService._allegro_controller.hand_pose(hand_joint_positions)

        controller_type = "JOINT_IMPEDANCE"
        controller_cfg = RobotService._deoxys_joint_impedance_controller_cfg

        # Send action to the robot
        RobotService._robot.control(
            controller_type=controller_type,
            action=arm_joint_positions.tolist() + [-1.0],
            controller_cfg=controller_cfg
        )

        return service_pb2.Result(ok=service_pb2.Ok())

    def ControlArmEEPoseHandDeltaJointPositions(self, request, context):

        if RobotService._robot is None:
            logger.error("Robot not started")
            return service_pb2.Result(err=service_pb2.Err(message="Robot not started"))

        if RobotService._allegro_controller is None:
            logger.error("Allegro hand not started")
            return service_pb2.Result(err=service_pb2.Err(message="Allegro hand not started"))

        hand_delta_joint_positions = np.array(
            request.joint_positions)
        RobotService._allegro_controller.hand_pose(
            hand_delta_joint_positions, absolute=False)

        controller_type = "OSC_POSE"
        controller_cfg = RobotService._deoxys_osc_controller_cfg_no_delta

        ee_pose = np.array(request.pose)

        ee_trans = ee_pose[:3]
        ee_rpy = ee_pose[3:]
        ee_quat = rpy_to_quaternion(ee_rpy)

        action_pos = ee_trans
        action_ori = transform_utils.quat2axisangle(ee_quat)

        # Construct the action vector
        action = np.concatenate((action_pos, action_ori, [-1.0]))

        # Send action to the robot
        RobotService._robot.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg
        )
        return service_pb2.Result(ok=service_pb2.Ok())

    def ControlArmEEPoseHandJointPositions(self, request, context):

        if RobotService._robot is None:
            logger.error("Robot not started")
            return service_pb2.Result(err=service_pb2.Err(message="Robot not started"))

        if RobotService._allegro_controller is None:
            logger.error("Allegro hand not started")
            return service_pb2.Result(err=service_pb2.Err(message="Allegro hand not started"))

        hand_joint_positions = np.array(
            request.joint_positions)
        RobotService._allegro_controller.hand_pose(
            hand_joint_positions, absolute=True)

        controller_type = "OSC_POSE"
        controller_cfg = RobotService._deoxys_osc_controller_cfg_no_delta

        ee_pose = np.array(request.pose)

        ee_trans = ee_pose[:3]
        ee_rpy = ee_pose[3:]
        ee_quat = rpy_to_quaternion(ee_rpy)

        action_pos = ee_trans
        action_ori = transform_utils.quat2axisangle(ee_quat)

        # Construct the action vector
        action = np.concatenate((action_pos, action_ori, [-1.0]))

        # Send action to the robot
        RobotService._robot.control(
            controller_type=controller_type,
            action=action,
            controller_cfg=controller_cfg
        )
        return service_pb2.Result(ok=service_pb2.Ok())
