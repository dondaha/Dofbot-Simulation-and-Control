

from dofbot import DofbotEnv
import numpy as np
import time
import copy
from scipy.spatial.transform import Rotation as R
import time
import math


if __name__ == '__main__':
    env = DofbotEnv()
    env.reset()
    Reward = False


    '''
    constants here
    '''
    GRIPPER_DEFAULT_ANGLE = 20. / 180. * 3.1415
    GRIPPER_CLOSE_ANGLE = -20. / 180. * 3.1415

    # define state machine
    INITIAL_STATE = 0
    GRASP_STATE = 1
    LIFT_STATE = 2
    PUT_STATE = 3
    MOVE_STATE = 4
    BACK_STATE = 5
    current_state = INITIAL_STATE


    initial_jointposes = [1.57, 0., 1.57, 1.57, 1.57]

    # offset to grasp object
    obj_offset = [-0.023, -0.023, 0.09]
    obj_offset2 = [-0.032, 0.032, 0.13]
    obj_offset3 = [-0.025, 0.025, 0.09]

    block_pos, block_orn = env.get_block_pose()

    start_time = None

    while not Reward:



        '''
        #获取物块位姿、目标位置和机械臂位姿，计算机器臂关节和夹爪角度，使得机械臂夹取绿色物块，放置到紫色区域。
        '''

        '''
        code here
        '''

        if current_state == INITIAL_STATE:
            target_pos = copy.deepcopy(block_pos)
            target_pos = (target_pos[0] + obj_offset[0], target_pos[1] + obj_offset[1], target_pos[2] + obj_offset[2])
            target_orn = copy.deepcopy(block_orn)

            eular_pos = R.from_quat(target_orn).as_euler('xyz', degrees=False)
            eular_pos[1] = -3.1415 / 2
            target_orn = R.from_euler('xyz', eular_pos).as_quat()

            target_joint_state = env.dofbot_setInverseKine(target_pos, block_orn * -1)
            env.dofbot_control(target_joint_state, GRIPPER_DEFAULT_ANGLE)

            # check if reach target pos
            current_joint_state, gripper_angle = env.get_dofbot_jointPoses()
            if (np.sum(np.isclose(np.array(current_joint_state), np.array(target_joint_state), atol = 1e-2)) == 5):
                current_state = GRASP_STATE
            
            print(target_joint_state, current_joint_state)
        elif current_state == GRASP_STATE:
            target_pos = copy.deepcopy(block_pos)
            target_pos = (target_pos[0] + obj_offset[0], target_pos[1] + obj_offset[1], target_pos[2] + obj_offset[2])
            target_orn = copy.deepcopy(block_orn)

            eular_pos = R.from_quat(target_orn).as_euler('xyz', degrees=False)
            eular_pos[1] = -3.1415 / 2
            target_orn = R.from_euler('xyz', eular_pos).as_quat()

            target_joint_state = env.dofbot_setInverseKine(target_pos, block_orn * -1)
            env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)

            current_joint_state, gripper_angle = env.get_dofbot_jointPoses()

            if start_time is None:
                start_time = time.time()
            current_time = time.time()
            if (current_time - start_time > 2.0):
                current_state = LIFT_STATE
                start_time = None
        elif current_state == LIFT_STATE:
            target_pos = copy.deepcopy(block_pos)
            target_pos = (target_pos[0] + obj_offset[0], target_pos[1] + obj_offset[1], target_pos[2] + obj_offset[2] + 0.05)
            target_orn = copy.deepcopy(block_orn)

            eular_pos = R.from_quat(target_orn).as_euler('xyz', degrees=False)
            eular_pos[1] = -3.1415 / 2
            target_orn = R.from_euler('xyz', eular_pos).as_quat()

            target_joint_state = env.dofbot_setInverseKine(target_pos, block_orn * -1)
            env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)

            # check if reach target pos
            current_joint_state, gripper_angle = env.get_dofbot_jointPoses()
            if (np.sum(np.isclose(np.array(current_joint_state), np.array(target_joint_state), atol = 1e-2)) == 5):
                current_state = MOVE_STATE
        elif current_state == MOVE_STATE:
            target_pos = env.get_target_pose()
            # TODO: fix block pos
            target_pos = (target_pos[0] + obj_offset2[0], target_pos[1] + obj_offset2[1], block_pos[2] + obj_offset2[2])
            target_orn = copy.deepcopy(block_orn)
            

            eular_pos = R.from_quat(target_orn).as_euler('xyz', degrees=False)
            eular_pos[1] = -3.1415 / 2
            eular_pos[2] = -eular_pos[2]
            target_orn = R.from_euler('xyz', eular_pos).as_quat()

            target_joint_state = env.dofbot_setInverseKine(target_pos, block_orn * -1)
            env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)

            # check if reach target pos
            current_joint_state, gripper_angle = env.get_dofbot_jointPoses()
            if (np.sum(np.isclose(np.array(current_joint_state), np.array(target_joint_state), atol = 1e-2)) == 5):
                current_state = BACK_STATE
        elif current_state == BACK_STATE:
            if start_time is None:
                start_time = time.time()
            current_time = time.time()

            target_pos = env.get_target_pose()
            # TODO: fix block pos
            target_pos = (target_pos[0] + obj_offset3[0], target_pos[1] + obj_offset3[1], block_pos[2] + obj_offset3[2])
            target_orn = copy.deepcopy(block_orn)
            

            eular_pos = R.from_quat(target_orn).as_euler('xyz', degrees=False)
            eular_pos[1] = -3.1415 / 2
            eular_pos[2] = -eular_pos[2]
            target_orn = R.from_euler('xyz', eular_pos).as_quat()

            target_joint_state = env.dofbot_setInverseKine(target_pos, block_orn * -1)

            if (current_time - start_time > 2.0):
                env.dofbot_control(target_joint_state, GRIPPER_DEFAULT_ANGLE)
            else:
                env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)


        Reward = env.reward()
