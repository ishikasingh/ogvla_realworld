from arm_hand_deployment.consts import CONFIG_PATH

import os

import hydra
from omegaconf import DictConfig, OmegaConf

from typing import Optional

from arm_hand_deployment.enums import ArmHandType


@hydra.main(config_path=CONFIG_PATH, config_name="config", version_base="1.3")
def main(cfg: DictConfig):

    arm_hand_type = ArmHandType(cfg.arm_hand_type)
    print(arm_hand_type)

    if arm_hand_type == ArmHandType.FRANKA:
        from arm_hand_deployment.franka import run_server
    else:
        raise NotImplementedError(
            f"ArmHandType {arm_hand_type} not implemented")
    run_server(cfg)


if __name__ == '__main__':
    main()
