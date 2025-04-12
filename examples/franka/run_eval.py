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
from queue import Queue
import threading
import time
import shutil
import pickle
import subprocess
import json

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
        self.data = []
        self.pipeline = None
        
    def start(self):
        self.is_running = True
        self.thread = threading.Thread(target=self._capture_stream)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        self.is_running = False
        self.data = []
        if self.pipeline:
            self.pipeline.stop()
        self.thread.join()

    def get_images(self):
        if not self.queue.empty():
            data = self.queue.get()
            return data
        else:
            return None
        
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

        # for _ in range(30):
        #     frames = pipeline.wait_for_frames()
        
        # Main capture loop
        if os.path.exists('tmp'):
            shutil.rmtree('tmp')
        os.makedirs('tmp', exist_ok=True)
        self.data= []
        while self.is_running:
            try:
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
            except RuntimeError as e:
                print(f"[ERROR] Frame alignment failed: {e}")
                # self.queue = Queue()
                continue

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
                
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            print("Color Image Shape: ", color_image.shape)
            print("Depth Image Shape: ", depth_image.shape)
            
            timestamp = time.time()

            print("Timestamp: ", timestamp)

            cv2.imwrite(f'tmp/color_{timestamp}.png', cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
            cv2.imwrite(f'tmp/depth_{timestamp}.png', depth_image)
            self.data.append({
                'timestamp': timestamp,
                # 'color': color_image,
                # 'depth': depth_image
            })
            # cv2.imwrite(f'color_{frame_id}.png', cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
            
        pipeline.stop()


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


def run_eval_step(client, camera_streams, data, task_name, episode_num, step):
    is_moving = threading.Event()
    camera_streams[0].start()
    last_camera_data = []
    while len(camera_streams[0].data) == 0:
        continue
    camera_streams[0].stop()
    last_camera_data = camera_streams[0].data[-1]
    color_image_path = f'tmp/color_{last_camera_data["timestamp"]}.png'
    depth_image_path = f'tmp/depth_{last_camera_data["timestamp"]}.png'
    obs_input = {
        'color': cv2.imread(color_image_path),
        'depth': cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED),
        'task_name': task_name,
        'episode_num': episode_num,
        'step': step,
    }
    pickle_file_path = '/home/yiyang/hsc/ogvla_realworld/data/eval/obs_input.pkl'

    # Save data to a pickle file
    with open(pickle_file_path, 'wb') as f:
        pickle.dump(obs_input, f)

    # Step 2: Add the file to Git
    subprocess.run(['git', 'add', pickle_file_path])

    # Step 3: Commit the changes
    commit_message = "Add obs_input pickle file"
    subprocess.run(['git', 'commit', '-m', commit_message])

    # Step 4: Push the changes to the repository
    subprocess.run(['git', 'push', 'origin', 'main'])

    input("Press Enter to continue...")
    
    subprocess.run(['git', 'pull', 'origin', 'main'])

    action_path = f'/home/yiyang/hsc/ogvla_realworld/data/eval/action'
    with open(action_path + '.json', 'r') as f:
        action_data = json.load(f)

    keypoint = action_data['keypoint']
    gripper_state = action_data['gripper_state']

    def move_robot():
        try:
            is_moving.set()
            client.MoveToEndEffectorPose(keypoint)
            client.SetGripperAction(gripper_state)
        finally:
            is_moving.clear()
    
    # def collect_data():
    #     # camera_streams = [camera_stream.start() for camera_stream in camera_streams]
    #     while is_moving.is_set() or any(not cam.queue.empty() for cam in camera_streams):
    #         # Only collect when all cameras have images
    #         if all(not cam.queue.empty() for cam in camera_streams):
    #             images = {f'cam_{i}': cam.get_images() for i, cam in enumerate(camera_streams)}
                
    #         # time.sleep(0.001)  # Small sleep to prevent busy waiting
        # get_all_camera_images()
    
    camera_streams[0].start()
    time.sleep(1)  # Allow some time for the camera to start
    # Start threads
    robot_thread = threading.Thread(target=move_robot)
    # collect_thread = threading.Thread(target=collect_data)
    
    robot_thread.start()
    # collect_thread.start()
    print("Robot thread started")
    
    # Wait for completion
    robot_thread.join()
    # collect_thread.join()
    print("Robot thread finished")
    time.sleep(1)  
    camera_streams[0].stop()

    camera_data_list = camera_streams[0].data.copy()

    for i, camera_data in enumerate(camera_data_list):
        color_image_path = f'tmp/color_{camera_data["timestamp"]}.png'
        depth_image_path = f'tmp/depth_{camera_data["timestamp"]}.png'

        # import pdb; pdb.set_trace()


        if os.path.exists(color_image_path) and os.path.exists(depth_image_path):
            color_image = cv2.imread(color_image_path)
            depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

            camera_data_list[i].update({
            'color': color_image,
            'depth': depth_image
            })
        else:
            camera_data_list = camera_data_list[:i]
            break

    # import pdb; pdb.set_trace()
    data_point = {
                    'gripper_state': gripper_state,
                    'images': camera_data_list,
                    'ee_pose': client.GetEndEffectorPose(),
                    'keypoint_pose': keypoint,
                }
    data.append(data_point)
    
    return data

@hydra.main(config_path=CONFIG_PATH, config_name="config", version_base="1.3")
def main(cfg: DictConfig):
    print(OmegaConf.to_yaml(cfg))

    port: int = int(cfg.rpc_port)
    server_ip: str = cfg.server_ip

    logger.info(f"Connecting to server {server_ip}:{port}")

    camera = CameraStream()

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
        
        # import pdb; pdb.set_trace()
        # Get task name and episode number from user
        # task_name = input("Enter task name: ").strip()
        # while not task_name:
        #     print("Task name cannot be empty")
        #     task_name = input("Enter task name: ").strip()

        task_name = 'pickup cube'
            
        episode_num = input("Enter episode number: ").strip()
        while not episode_num.isdigit():
            print("Episode number must be a positive integer")
            episode_num = input("Enter episode number: ").strip()
        episode_num = int(episode_num)

        # keypoints = []
        # # keypoints.append(client.GetEndEffectorPose())

        # while True:
        #     key = input("Press Enter to continue to next frame, or 'q' to quit: ")
        #     if key.lower() == 'q':
        #         break
        #     keypoints.append(client.GetEndEffectorPose())
        
        # # get last keypoint
        # keypoints.append(client.GetEndEffectorPose())

        # # print keypoints
        # print("Keypoints:")
        # for i, keypoint in enumerate(keypoints):
        #     print(f"Keypoint {i}: {keypoint}")

        # record trajectory

        data = []
        while True:
            # gripper_input = input("Enter gripper state (0: open, 1: closed): ")
            # gripper_state = float(gripper_input.strip())
            # if gripper_state == 0:
            #     gripper_state = -1
            # assert gripper_state in [-1, 1], "Gripper state must be -1 or 1"
            
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

            data = run_eval_step(
                client=client,
                camera_streams=[camera],
                data=data
            )
            success = input("Was the step successful? (y/n): ").strip().lower()
            if success == 'n':
                print("Step marked as unsuccessful. Exiting.")
                break
            # camera.stop()



        # save trajectory
        with open(f'/home/yiyang/hsc/ogvla_realworld/data/eval/{task_name}_trajectory_{episode_num}.pkl', 'wb') as f:
            pickle.dump(data, f)

        # with open(f'/home/yiyang/hsc/ogvla_realworld/data/train/{task_name}_trajectory_{episode_num}.pkl', 'rb') as f:
        #     data = pickle.load(f)            

        # import pdb; pdb.set_trace()


        # Create a video writer
        video_path = f'/home/yiyang/hsc/ogvla_realworld/data/eval/{task_name}_trajectory_{episode_num}.mp4'
        frame_height, frame_width, _ = data[0]['images'][0]['color'].shape
        fps = 30  # Frames per second
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(video_path, fourcc, fps, (frame_width, frame_height))

        # Write frames to the video
        for i, frame_data in enumerate(data):
            for frames in data[i]['images']:
                if 'color' not in frames.keys():
                    break
                color_frame = frames['color']
                # color_frame = cv2.cvtColor(color_frame, cv2.COLOR_GRAY2BGR)
                video_writer.write(color_frame)
                print(f"Writing frame {i + 1}/{len(data)}")

        # Release the video writer
        video_writer.release()
        print(f"Video saved to {video_path}")

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

        # with open(f'/home/yiyang/hsc/ogvla_realworld/data/train/pickup_cube_trajectory_1.pkl', 'rb') as f:
        #     data = pickle.load(f)            

        # import pdb; pdb.set_trace()


        # # Create a video writer
        # video_path = f'/home/yiyang/hsc/ogvla_realworld/data/train/pickup_cube_depth_trajectory_1.mp4'
        
        # frame_height, frame_width, _ = data[0]['images'][0]['color'].shape
        # fps = 30  # Frames per second
        # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        # video_writer = cv2.VideoWriter(video_path, fourcc, fps, (frame_width, frame_height))

        # # Write frames to the video
        # for i, frame_data in enumerate(data):
        #     for frames in data[i]['images']:
        #         if 'color' not in frames.keys():
        #             break

        #         color_frame = frames['depth']
        #         import pdb; pdb.set_trace()
        #         # color_frame = cv2.cvtColor(color_frame, cv2.COLOR_RGB2BGR)
        #         color_frame = cv2.cvtColor(color_frame, cv2.COLOR_GRAY2BGR)
        #         video_writer.write(color_frame)
        #         print(f"Writing frame {i + 1}/{len(data)}")

        # # Release the video writer
        # video_writer.release()
        # print(f"Video saved to {video_path}")
    # main()
