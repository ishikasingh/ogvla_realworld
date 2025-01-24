from arm_hand_deployment.franka_allegro.proto import service_pb2, service_pb2_grpc
from arm_hand_deployment.interfaces.client import Client

import numpy as np

from loguru import logger

from typing import Iterable


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

    def GetEndEffectorPose(self) -> np.ndarray:
        result = self._stub.GetEndEffectorPose(service_pb2.Empty())
        return np.array(result.pose)

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
