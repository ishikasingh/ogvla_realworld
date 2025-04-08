from arm_hand_deployment.consts import CONFIG_PATH

import hydra
from omegaconf import DictConfig, OmegaConf

from loguru import logger

from time import sleep, time

import numpy as np

from arm_hand_deployment.utils.np_3d_utils import rpy_to_quaternion, quaternion_to_rpy, quaternion_multiply
from arm_hand_deployment.utils.client_context import robot_client_context

from arm_hand_deployment.franka.communication.client import FrankaClient

import cv2
import open3d as o3d
import os

from real_world_perception.cameras import *
from real_world_perception.read_real_data import get_camera_image, get_pointcloud_multiview

def get_all_camera_images():
    file_dir = 'data'
    scene = 'test'
    os.makedirs(f'{file_dir}/{scene}', exist_ok=True)

    cams = [RIGHT_CAMERA] #LEFT_CAMERA, 
    cam_images = []
    for cam in cams:
        color, depth = get_camera_image(cam)
        color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        cv2.imwrite(f'{file_dir}/{scene}/color_{cam}.png', color)
        cv2.imwrite(f'{file_dir}/{scene}/depth_{cam}.png', depth)
        cam_images.append((color, depth))
    return cam_images

def test_pointcloud_multiview():
    file_dir = 'data'
    scene = 'cube'
    os.makedirs(f'{file_dir}/{scene}', exist_ok=True)

    cams = [LEFT_CAMERA, RIGHT_CAMERA]
    object_list = ['cube']
    pcd, _ = get_pointcloud_multiview(cams, object_list, data_path=f'{file_dir}/{scene}', icp=False)
    o3d.io.write_point_cloud(f'{file_dir}/{scene}/pcd.ply', pcd)


def timing_decorator(func):
    def wrapper(*args, **kwargs):
        start_time = time()  # Record start time
        result = func(*args, **kwargs)  # Call the function
        end_time = time()  # Record end time
        execution_time = end_time - start_time  # Calculate execution time
        print(
            f"Execution time of {func.__name__}: {execution_time:.4f} seconds")
        return result

    return wrapper


@hydra.main(config_path=CONFIG_PATH, config_name="config", version_base="1.3")
def main(cfg: DictConfig):
    print(OmegaConf.to_yaml(cfg))

    port: int = int(cfg.rpc_port)
    server_ip: str = cfg.server_ip

    logger.info(f"Connecting to server {server_ip}:{port}")

    with robot_client_context(server_ip, port, FrankaClient) as client:
        client: FrankaClient

        # 7 joint positions for the arm + 1 for the gripper (width of the gripper)
        positions = client.GetJointPositions()
        target_joint_positions = [
            0.09162008114028396,
            -0.19826458111314524,
            -0.01990020486871322,
            -2.4732269941140346,
            -0.01307073642274261,
            2.30396583422025,
            0.8480939705504309,
        ]
        assert client.MoveToJointPositions(target_joint_positions)

        # Get task name and episode number from user
        task_name = input("Enter task name: ").strip()
        while not task_name:
            print("Task name cannot be empty")
            task_name = input("Enter task name: ").strip()
            
        episode_num = input("Enter episode number: ").strip()
        while not episode_num.isdigit():
            print("Episode number must be a positive integer")
            episode_num = input("Enter episode number: ").strip()
        episode_num = int(episode_num)

        keypoints = []
        while True:
            keypoints.append(client.GetEndEffectorPose())
            key = input("Press Enter to continue to next frame, or 'q' to quit: ")
            if key.lower() == 'q':
                break

        # record trajectory
        data = []
        for i, keypoint in enumerate(keypoints):
            gripper_input = input("Enter gripper state (0: open, 1: closed): ")
            gripper_state = float(gripper_input.strip())
            if gripper_state == 0:
                gripper_state = -1
            assert gripper_state in [-1, 1], "Gripper state must be -1 or 1"
            
            joint_traj = client.MoveToEndEffectorPose(keypoint, return_joint_traj=True)

            for joint_pos in joint_traj:
                data.append({
                    'gripper_state': gripper_state,
                    # somehow get the joint_traj out here for intermediate poses
                    'images': get_all_camera_images(),
                    'ee_pose': client.GetEndEffectorPose(),
                    'keypoint_pose': keypoint,
                    'joint_pos': joint_pos
                })
                assert client.MoveToJointPositions(joint_pos)
            
            client.SetGripperAction(gripper_state)

        # save trajectory
        np.save(f'data/train/{task_name}_trajectory_{episode_num}.npy', data)
            

        import pdb; pdb.set_trace()


        

        # @timing_decorator
        # def execute(action):
        #     assert client.ControllDeltaEndEffectorPose(action=action)

        # # warmup
        # execute([0, 0, 0, 0, 0, 0])

        # print('-'*80)

        # # deoxys's robot interface will block until it has been at least 1/control_freq since last action
        # # but it will be better if we ensure this on our end.

        # print('+z')
        # for i in range(20):
        #     action = [0, 0, 0.01, 0, 0, 0]
        #     execute(action)
            

        # print('-z')
        # for i in range(20):
        #     action = [0, 0, -0.01, 0, 0, 0]
        #     execute(action)

        # print('+x')
        # for i in range(20):
        #     action = [0.02, 0, 0, 0, 0, 0]
        #     execute(action)

        # print('-x')
        # for i in range(20):
        #     action = [-0.02, 0, 0, 0, 0, 0]
        #     execute(action)


if __name__ == '__main__':
    main()
