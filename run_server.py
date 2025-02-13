from arm_hand_deployment.consts import CONFIG_PATH

import os

import hydra
from omegaconf import DictConfig, OmegaConf

from typing import Optional

from arm_hand_deployment.enums import ArmHandType


@hydra.main(config_path=CONFIG_PATH, config_name="config", version_base="1.3")
def main(cfg: DictConfig):

    try:
        arm_hand_type = ArmHandType(cfg.arm_hand_type)
    except ValueError:
        raise ValueError(
            f"Invalid arm_hand_type: {cfg.arm_hand_type}. "
            f"Valid values are: {[e.value for e in ArmHandType]}"
        )
    print(arm_hand_type)

    if arm_hand_type == ArmHandType.FRANKA:
        from arm_hand_deployment.franka.run_server import run_server
    elif arm_hand_type == ArmHandType.FRANKA_ALLEGRO:
        from arm_hand_deployment.franka_allegro.run_server import run_server
    else:
        raise NotImplementedError(
            f"ArmHandType {arm_hand_type} not implemented")
    run_server(cfg)


if __name__ == '__main__':
    main()
