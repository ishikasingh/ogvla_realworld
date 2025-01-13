

from omegaconf import DictConfig


from concurrent import futures

import grpc

from arm_hand_deployment.franka.proto import service_pb2_grpc

from arm_hand_deployment.franka.communication.server import RobotService

from arm_hand_deployment.consts import CONFIG_PATH

from loguru import logger

import os


def run_server(cfg: DictConfig):
    port: int = int(cfg.rpc_port)

    fci_ip: str = cfg.fci_ip
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))

    deoxys_general_cfg_file = os.path.join(
        CONFIG_PATH, 'franka', 'deoxys_config.yaml')
    deoxys_osc_controller_cfg_file = os.path.join(
        CONFIG_PATH, 'franka', 'osc-pose-controller.yaml')
    deoxys_joint_position_controller_cfg_file = os.path.join(
        CONFIG_PATH, 'franka', 'joint-position-controller.yaml')
    deoxys_joint_impedance_controller_cfg_file = os.path.join(
        CONFIG_PATH, 'franka', 'joint-impedance-controller.yaml')

    service_pb2_grpc.add_FrankaServiceServicer_to_server(
        RobotService(fci_ip,
                     deoxys_general_cfg_file=deoxys_general_cfg_file,
                     deoxys_osc_controller_cfg_file=deoxys_osc_controller_cfg_file,
                     deoxys_joint_position_controller_cfg_file=deoxys_joint_position_controller_cfg_file,
                     deoxys_joint_impedance_controller_cfg_file=deoxys_joint_impedance_controller_cfg_file,
                     ), server)
    server.add_insecure_port(f"[::]:{port}")
    server.start()
    logger.info(f"Server started, listening on {port}")
    server.wait_for_termination()
