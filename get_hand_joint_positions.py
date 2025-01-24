from allegro_hand.controller import AllegroController

import numpy as np

from time import sleep

controller = AllegroController()

sleep(1)

# target_hand_pose = np.array([0., 0.7, 0., 0.4, 0., 0.7, 0., 0.4, 0., 0.7, 0., 0.4, 1.,
#                      0., 0., 0.])

# controller.hand_pose(target_hand_pose)

sleep(1)

print(controller.current_joint_pose.position)