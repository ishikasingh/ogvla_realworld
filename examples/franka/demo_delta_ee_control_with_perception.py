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

def get_camera_image(cam): # L515 or D435
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(cam)

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    found_rgb = False
    for s in device.sensors:
        print(s.get_info(rs.camera_info.name))
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)
        
    config.enable_stream(rs.stream.depth, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, rs.format.rgb8, 30)

    profile = pipeline.start(config)
    stream = profile.get_streams()  
    # for s in stream:
    #     intr = s.as_video_stream_profile().get_intrinsics()
    #     print(cam, intr)

    color_sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
    color_sensor.set_option(rs.option.exposure, 1000)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    # print("Depth Scale: ", depth_scale)

    # We will be removing the background of objects more than clipping_distance_in_meters meters away
    clipping_distance_in_meters = 2 # 2 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    align_to = rs.stream.color
    align = rs.align(align_to)

    for _ in range(30):
        pipeline.wait_for_frames() 

    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame() 
    color_frame = aligned_frames.get_color_frame()

    intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
    print(cam, intrinsics)

    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(aligned_depth_frame.get_data())

    pipeline.stop()

    return color_image, depth_image


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
        
        # xyz+rpy
        for i in range(10):
            data = {}
            cam_images = get_all_camera_images()
            data['images'] = cam_images
            ee_pose = client.GetEndEffectorPose()
            data['ee_pose'] = ee_pose
            print(f"End effector pose: {ee_pose}")
            # save data
            np.save(f'data/eval/data_{i}.npy', data)
            # # Get keyboard input from user
            # key = input("Press Enter to continue to next frame, or 'q' to quit: ")
            # if key.lower() == 'q':
            #     break

            import pdb; pdb.set_trace()


        

        @timing_decorator
        def execute(action):
            assert client.ControllDeltaEndEffectorPose(action=action)

        # warmup
        execute([0, 0, 0, 0, 0, 0])

        print('-'*80)

        # deoxys's robot interface will block until it has been at least 1/control_freq since last action
        # but it will be better if we ensure this on our end.

        print('+z')
        for i in range(20):
            action = [0, 0, 0.01, 0, 0, 0]
            execute(action)
            

        print('-z')
        for i in range(20):
            action = [0, 0, -0.01, 0, 0, 0]
            execute(action)

        print('+x')
        for i in range(20):
            action = [0.02, 0, 0, 0, 0, 0]
            execute(action)

        print('-x')
        for i in range(20):
            action = [-0.02, 0, 0, 0, 0, 0]
            execute(action)


if __name__ == '__main__':
    main()
