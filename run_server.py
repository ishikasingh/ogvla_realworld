from arm_hand_deployment.consts import CONFIG_PATH

import os

import hydra
from omegaconf import DictConfig, OmegaConf

from loguru import logger

from concurrent import futures

import grpc

from arm_hand_deployment.franka.proto import service_pb2_grpc

from typing import Optional

from arm_hand_deployment.franka.communication.server import RobotService


@hydra.main(config_path=CONFIG_PATH, config_name="config", version_base="1.3")
def main(cfg: DictConfig):
    print(OmegaConf.to_yaml(cfg))
    port: int = int(cfg.rpc_port)

    fci_ip: str = cfg.fci_ip
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))

    # TODO: specify deoxys config files in config.yaml
    service_pb2_grpc.add_FrankaServiceServicer_to_server(
        RobotService(fci_ip,
                     deoxys_general_cfg_file=os.path.join(
                         CONFIG_PATH, "deoxys_config.yaml"),
                     deoxys_osc_controller_cfg_file=os.path.join(
                         CONFIG_PATH, "osc-pose-controller.yaml"),
                     deoxys_joint_position_controller_cfg_file=os.path.join(CONFIG_PATH,
                                                                            "joint-position-controller.yaml"),
                     deoxys_joint_impedance_controller_cfg_file=os.path.join(CONFIG_PATH,
                                                                             "joint-impedance-controller.yaml"),
                     ), server)
    server.add_insecure_port(f"[::]:{port}")
    server.start()
    logger.info(f"Server started, listening on {port}")
    server.wait_for_termination()


if __name__ == '__main__':
    main()
