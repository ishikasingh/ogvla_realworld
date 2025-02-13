from allegro_hand.controller import AllegroController

import numpy as np

from time import sleep

controller = AllegroController()

sleep(1)

# target_hand_pose = np.array([-0.03941083,  0.97284402,  0.60625225,  1.64531358,  0.06266678,
#                              1.04261185,  0.53825968,  1.77801447, -0.0705667,  0.69368394,
#                              1.54838423, -0.03000194,  1.53817646,  0.34333583, -0.22883138,
#                              1.67265262])


target_hand_pose = np.array([0., 0.7, 0., 0.4, 0., 0.7, 0., 0.4, 0., 0.7, 0., 0.4, 1.,
                             0., 0., 0.])

target_hand_pose[1]=1.71 #-0.296
target_hand_pose[5]=1.71 #-0.296
target_hand_pose[9]=1.71 #-0.296

# Test grav comp
controller.apply_grav_comp([0.0, 0.0, -9.81])

controller.hand_pose(target_hand_pose)

sleep(1)

print(controller.current_joint_pose.position)
