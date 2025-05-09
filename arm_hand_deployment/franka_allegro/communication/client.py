from arm_hand_deployment.franka_allegro.proto import service_pb2, service_pb2_grpc
from arm_hand_deployment.interfaces.client import Client

import numpy as np

from loguru import logger

from typing import Iterable, Sequence


class FrankaAllegroClient(Client):
    def __init__(self, channel):
        self._stub = service_pb2_grpc.FrankaAllegroServiceStub(channel)

    def Start(self) -> bool:
        result = self._stub.Start(service_pb2.Empty())
        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def Stop(self) -> bool:
        result = self._stub.Stop(service_pb2.Empty())
        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def GetJointPositions(self) -> np.ndarray:
        result = self._stub.GetJointPositions(service_pb2.Empty())
        return np.array(result.positions)

    def GetEndEffectorPose(self) -> np.ndarray:
        result = self._stub.GetEndEffectorPose(service_pb2.Empty())
        return np.array(result.pose)
    
    def SetGravityVector(self, gravity_vector: Iterable[float]) -> bool:
        """
        Sets the gravity vector for gravity compensation.
        :param gravity_vector: A sequence of 3 floats representing the gravity vector [x, y, z].
        :return: True if successful, False otherwise.
        """
        if len(gravity_vector) != 3:
            logger.error("Gravity vector must be a list of 3 floats (x, y, z).")
            return False

        result = self._stub.SetGravityVector(
            service_pb2.GravityVector(vector=gravity_vector))

        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def MoveToJointPositions(self, joint_positions: np.ndarray) -> bool:
        # Convert joint positions to protobuf message format
        joint_positions_msg = service_pb2.JointPositions(
            positions=joint_positions.tolist())
        result = self._stub.MoveToJointPositions(joint_positions_msg)
        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def ArmMoveToJointPositions(self, positions: Iterable[float]) -> bool:
        result = self._stub.ArmMoveToJointPositions(
            service_pb2.JointPositions(positions=positions))
        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def ArmMoveToEndEffectorPose(self, pose: Iterable[float]) -> bool:
        result = self._stub.ArmMoveToEndEffectorPose(
            service_pb2.Pose(pose=pose))
        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def HandMoveToJointPositions(self, positions: Iterable[float]) -> bool:
        result = self._stub.HandMoveToJointPositions(
            service_pb2.JointPositions(positions=positions))
        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def ControlJointPositions(self, action: Iterable[float]) -> bool:
        """
        action: (23,) joint positions
        """
        result = self._stub.ControlJointPositions(
            service_pb2.JointPositions(positions=action))
        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def ControlArmEEPoseHandDeltaJointPositions(self, action: Sequence[float]) -> bool:
        """
        action: (22,) 6 end effector pose + 16 hand joint positions (delta)
        """

        assert len(action) == 22

        result = self._stub.ControlArmEEPoseHandDeltaJointPositions(
            service_pb2.ArmEEPoseHandJointPositions(
                pose=action[:6],
                joint_positions=action[6:],
            ))
        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def ControlArmEEPoseHandJointPositions(self, action: Sequence[float]) -> bool:
        """
        action: (22,) 6 end effector pose + 16 hand joint positions
        """

        assert len(action) == 22

        result = self._stub.ControlArmEEPoseHandJointPositions(
            service_pb2.ArmEEPoseHandJointPositions(
                pose=action[:6],
                joint_positions=action[6:],
            ))
        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False
