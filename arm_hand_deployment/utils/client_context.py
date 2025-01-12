from contextlib import contextmanager
from arm_hand_deployment.interfaces.client import Client
import grpc

from loguru import logger


@contextmanager
def robot_client_context(server_ip: str, port: int, client_cls):
    channel = grpc.insecure_channel(f"{server_ip}:{port}")
    client: Client = client_cls(channel)

    try:
        assert client.Start(), "Failed to start client"
        logger.info(f"Connected to server {server_ip}:{port}")
        yield client
    finally:
        if not client.Stop():
            logger.error("Failed to stop client")
        channel.close()
        logger.info(f"Disconnected from server {server_ip}:{port}")
