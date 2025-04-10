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
import pyrealsense2 as rs


from deoxys.utils.ik_utils import IKWrapper

from arm_hand_deployment.utils.np_3d_utils import (
    quaternion_to_rpy,
    rpy_to_quaternion,
    quaternion_multiply_translation,
    quaternion_inverse,
    rpy_to_rotation_matrix,
    quaternion_slerp)


class CameraStream:
    def __init__(self):
        self.cam_serial = RIGHT_CAMERA
        self.is_running = False
        self.queue = Queue()
        self.pipeline = None
        
    def start(self):
        self.is_running = True
        self.thread = threading.Thread(target=self._capture_stream)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        self.is_running = False
        if self.pipeline:
            self.pipeline.stop()
        self.thread.join()
        
    def _capture_stream(self):
        # Initialize RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.cam_serial)
        
        # Configure streams
        config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, rs.format.rgb8, 30)
        
        # Start streaming
        profile = pipeline.start(config)
        
        # Setup alignment
        align_to = rs.stream.color
        align = rs.align(align_to)
        
        # Configure camera settings
        color_sensor = pipeline.get_active_profile().get_device().query_sensors()[1]
        color_sensor.set_option(rs.option.exposure, 1000)
        
        # Main capture loop
        while self.is_running:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
                
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            timestamp = time.time()
            self.queue.put({
                'timestamp': timestamp,
                'color': color_image,
                'depth': depth_image
            })
            
        pipeline.stop()


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

def get_joint_traj(keypoint, current_joint_positions):
    # Initialize IKWrapper for inverse kinematics calculation
    ik_wrapper = IKWrapper()

    # Extract target pose (xyz + rpy) and convert rpy to rotation matrix
    target_position: np.ndarray = np.array(keypoint[:3])

    target_rpy = keypoint[3:]
    target_mat = rpy_to_rotation_matrix(target_rpy)

    # Compute IK to get joint trajectory to the target end-effector pose
    joint_traj, debug_info = ik_wrapper.ik_trajectory_to_target_pose(
        target_position, target_mat, current_joint_positions, num_points=3
    )

    if joint_traj is None:
        error_message = "IK failed to compute a valid trajectory."
        
        return error_message

    # Interpolate the trajectory for smooth motion
    # joint_traj = ik_wrapper.interpolate_dense_traj(joint_traj)
    return joint_traj


def move_robot(client, ee_pose):
    client.MoveToEndEffectorPose(ee_pose)

def collect_data_during_movement(client, joint_pos, camera_streams, gripper_state, keypoint, data):
    is_moving = threading.Event()
    
    def move_robot():
        try:
            is_moving.set()
            client.MoveToEndEffectorPose(keypoint)
        finally:
            is_moving.clear()
    
    def collect_data():
        while is_moving.is_set() or any(not cam.queue.empty() for cam in camera_streams):
            # Only collect when all cameras have images
            if all(not cam.queue.empty() for cam in camera_streams):
                images = {f'cam_{i}': cam.get_images() for i, cam in enumerate(camera_streams)}
                data_point = {
                    'gripper_state': gripper_state,
                    'images': images,
                    'ee_pose': client.GetEndEffectorPose(),
                    'keypoint_pose': keypoint,
                }
                data.append(data_point)
            time.sleep(0.001)  # Small sleep to prevent busy waiting
    
    # Start threads
    robot_thread = threading.Thread(target=move_robot)
    collect_thread = threading.Thread(target=collect_data)
    
    robot_thread.start()
    collect_thread.start()
    
    # Wait for completion
    robot_thread.join()
    collect_thread.join()
    
    return data

@hydra.main(config_path=CONFIG_PATH, config_name="config", version_base="1.3")
def main(cfg: DictConfig):
    print(OmegaConf.to_yaml(cfg))

    port: int = int(cfg.rpc_port)
    server_ip: str = cfg.server_ip

    logger.info(f"Connecting to server {server_ip}:{port}")

    camera = CameraStream()
    camera.start()

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
        assert client.SetGripperAction(-1)
        

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
        # keypoints.append(client.GetEndEffectorPose())

        while True:
            key = input("Press Enter to continue to next frame, or 'q' to quit: ")
            if key.lower() == 'q':
                break
            keypoints.append(client.GetEndEffectorPose())
        
        # get last keypoint
        keypoints.append(client.GetEndEffectorPose())

        # print keypoints
        print("Keypoints:")
        for i, keypoint in enumerate(keypoints):
            print(f"Keypoint {i}: {keypoint}")

        # record trajectory
        data = []
        for i, keypoint in enumerate(keypoints):
            gripper_input = input("Enter gripper state (0: open, 1: closed): ")
            gripper_state = float(gripper_input.strip())
            if gripper_state == 0:
                gripper_state = -1
            assert gripper_state in [-1, 1], "Gripper state must be -1 or 1"
            
            # current_joint_positions = client.GetJointPositions()
            # # import pdb; pdb.set_trace()
            # joint_traj = get_joint_traj(keypoint, current_joint_positions[:7].tolist())
            # assert client.MoveToEndEffectorPose(keypoint)
            
            # for i, joint_pos in enumerate(joint_traj):
            #     data.append({
            #         'gripper_state': gripper_state,
            #         # somehow get the joint_traj out here for intermediate poses
            #         'images': get_all_camera_images(),
            #         'ee_pose': client.GetEndEffectorPose(),
            #         'keypoint_pose': keypoint,
            #         'joint_pos': joint_pos
            #     })
            #     assert client.MoveToJointPositions(joint_pos)
            #     print(i)
            
            # client.SetGripperAction(gripper_state)

            # Collect data during movement
            data.append(collect_data_during_movement(
                client=client,
                camera_streams=[camera],
                gripper_state=gripper_state,
                keypoint=keypoint,
                data=data
            ))

        # save trajectory
        np.savez_compressed(f'/home/yiyang/hsc/ogvla_realworld/data/train/{task_name}_trajectory_{episode_num}.npz', data)
            

        # import pdb; pdb.set_trace()


        

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
