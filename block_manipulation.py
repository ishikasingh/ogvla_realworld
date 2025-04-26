import numpy as np
import torch
import cv2
from segment_anything import sam_model_registry, SamPredictor
from scipy.spatial.transform import Rotation as R
import transforms3d as t3d

class BlockManipulationSkills:
    def __init__(self, sam_checkpoint, device="cuda"):
        """Initialize block manipulation skills with SAM model"""
        # Initialize SAM
        self.sam = sam_model_registry["default"](checkpoint=sam_checkpoint)
        self.sam.to(device)
        self.predictor = SamPredictor(self.sam)
        self.device = device

    def _pixel_to_world(self, pixel_x, pixel_y, depth, camera_intrinsics, camera_extrinsics):
        """Convert pixel coordinates to world coordinates using depth"""
        fx = camera_intrinsics[0, 0]
        fy = camera_intrinsics[1, 1]
        cx = camera_intrinsics[0, 2]
        cy = camera_intrinsics[1, 2]
        
        # Back-project to camera coordinates
        z = depth
        x = (pixel_x - cx) * z / fx
        y = (pixel_y - cy) * z / fy
        point_camera = np.array([x, y, z, 1.0])
        
        # Transform to world coordinates
        point_world = camera_extrinsics @ point_camera
        return point_world[:3]

    def _get_block_pose(self, rgb_image, depth_image, camera_intrinsics, camera_extrinsics, click_point):
        """Get block pose using SAM and depth"""
        # Get SAM mask
        self.predictor.set_image(rgb_image)
        masks, scores, _ = self.predictor.predict(
            point_coords=np.array([click_point]),
            point_labels=np.array([1]),
            multimask_output=False
        )
        
        # Get mask center and orientation
        mask = masks[0]
        y_indices, x_indices = np.where(mask)
        center_x = int(np.mean(x_indices))
        center_y = int(np.mean(y_indices))
        
        # Get depth at center
        center_depth = depth_image[center_y, center_x]
        
        # Convert to world coordinates
        position = self._pixel_to_world(
            center_x, center_y, center_depth,
            camera_intrinsics, camera_extrinsics
        )
        
        # Get orientation (assuming blocks are axis-aligned)
        # You may want to improve this using PCA or other methods
        orientation = R.from_euler('xyz', [0, 0, 0]).as_quat()
        
        return position, orientation

    def pick_block(self, rgb_image, depth_image, camera_intrinsics, camera_extrinsics, block_click_point, gripper_height=0.1):
        """Pick a block at the specified click point"""
        position, orientation = self._get_block_pose(
            rgb_image, depth_image, camera_intrinsics, camera_extrinsics, block_click_point
        )
        
        # Add gripper height offset for pre-grasp
        pre_grasp_pos = position + np.array([0, 0, gripper_height])
        
        return {
            'pre_grasp_pose': {
                'position': pre_grasp_pos,
                'orientation': orientation
            },
            'grasp_pose': {
                'position': position,
                'orientation': orientation
            }
        }

    def place_block(self, rgb_image, depth_image, camera_intrinsics, camera_extrinsics, 
                   target_click_point, current_block_height=0.05, place_offset=0.01):
        """Place a block at the specified target location"""
        target_pos, target_orientation = self._get_block_pose(
            rgb_image, depth_image, camera_intrinsics, camera_extrinsics, target_click_point
        )
        
        # Add offset for block height and safety margin
        place_pos = target_pos + np.array([0, 0, current_block_height + place_offset])
        
        return {
            'pre_place_pose': {
                'position': place_pos + np.array([0, 0, 0.05]),  # Additional height for pre-place
                'orientation': target_orientation
            },
            'place_pose': {
                'position': place_pos,
                'orientation': target_orientation
            }
        }

    def stack_blocks(self, rgb_image, depth_image, camera_intrinsics, camera_extrinsics, 
                    bottom_click_point, top_click_point):
        """Stack one block on top of another"""
        # Get bottom block location
        bottom_pos, bottom_orientation = self._get_block_pose(
            rgb_image, depth_image, camera_intrinsics, camera_extrinsics, bottom_click_point
        )
        
        # Pick the top block
        pick_poses = self.pick_block(
            rgb_image, depth_image, camera_intrinsics, camera_extrinsics, top_click_point
        )
        
        # Calculate place position on top of bottom block
        stack_pos = bottom_pos + np.array([0, 0, 0.05])  # Assuming 5cm block height
        
        return {
            'pick': pick_poses,
            'place': {
                'pre_place_pose': {
                    'position': stack_pos + np.array([0, 0, 0.05]),
                    'orientation': bottom_orientation
                },
                'place_pose': {
                    'position': stack_pos,
                    'orientation': bottom_orientation
                }
            }
        }

    def unstack_blocks(self, rgb_image, depth_image, camera_intrinsics, camera_extrinsics, 
                      top_click_point, target_click_point):
        """Unstack a block to a target location"""
        # Pick the top block
        pick_poses = self.pick_block(
            rgb_image, depth_image, camera_intrinsics, camera_extrinsics, top_click_point
        )
        
        # Place at target location
        place_poses = self.place_block(
            rgb_image, depth_image, camera_intrinsics, camera_extrinsics, target_click_point
        )
        
        return {
            'pick': pick_poses,
            'place': place_poses
        } 