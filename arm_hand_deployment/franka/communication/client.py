from arm_hand_deployment.franka.proto import service_pb2, service_pb2_grpc
from arm_hand_deployment.interfaces.client import Client

import numpy as np

from loguru import logger

from typing import Iterable


class FrankaClient(Client):
    def __init__(self, channel):
        self._stub = service_pb2_grpc.FrankaServiceStub(channel)

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

    def MoveToJointPositions(self, positions: Iterable[float]) -> bool:
        result = self._stub.MoveToJointPositions(
            service_pb2.JointPositions(positions=positions))
        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def MoveToEndEffectorPose(self, pose: Iterable[float]) -> bool:
        result = self._stub.MoveToEndEffectorPose(
            service_pb2.Pose(pose=pose))
        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def SetGripperAction(self, action: float) -> bool:
        assert -1.0 <= action <= 1.0
        result = self._stub.SetGripperAction(
            service_pb2.GripperAction(value=action))

        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def ControllDeltaEndEffectorPose(self, action: Iterable[float]) -> bool:
        """
        action: (6,) delta xyz+delta rpy
        """

        result = self._stub.ControllDeltaEndEffectorPose(
            service_pb2.DeltaPose(delta_pose=action))

        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False

    def ControlJointPositions(self, action: Iterable[float]) -> bool:
        """
        action: (7,) joint positions
        """
        result = self._stub.ControlJointPositions(
            service_pb2.JointPositions(positions=action))
        if result.HasField("ok"):
            return True
        else:
            logger.error(f"Error: {result.err.message}")
            return False
