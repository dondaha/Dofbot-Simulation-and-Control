

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
            # 设置目标位置
            target_pos = (block_pos[0] + obj_offset[0], block_pos[1] + obj_offset[1], block_pos[2] + obj_offset[2])
            # 计算目标位置对应的关节角度
            target_joint_state = env.dofbot_setInverseKine(target_pos)
            # 控制机械臂到目标位置
            env.dofbot_control(target_joint_state, GRIPPER_DEFAULT_ANGLE)
            # 检查是否到达目标位置
            current_joint_state, gripper_angle = env.get_dofbot_jointPoses()
            if (np.sum(np.isclose(np.array(current_joint_state), np.array(target_joint_state), atol = 1e-2)) == 5):
                current_state = GRASP_STATE
        elif current_state == GRASP_STATE:
            # 设置目标位置
            target_pos = (block_pos[0] + obj_offset[0], block_pos[1] + obj_offset[1], block_pos[2] + obj_offset[2])
            # 计算目标位置对应的关节角度
            target_joint_state = env.dofbot_setInverseKine(target_pos)
            # 控制机械臂到目标位置并抓取
            env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)
            # 1s后进入下一个状态
            if start_time is None:
                start_time = time.time()
            current_time = time.time()
            if (current_time - start_time > 1.0):
                current_state = LIFT_STATE
                start_time = None
        elif current_state == LIFT_STATE:
            # 设置目标位置
            target_pos = (block_pos[0] + obj_offset[0], block_pos[1] + obj_offset[1], block_pos[2] + obj_offset[2] + 0.05)
            # 计算目标位置对应的关节角度
            target_joint_state = env.dofbot_setInverseKine(target_pos)
            env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)
            # 检查是否到达目标位置
            current_joint_state, gripper_angle = env.get_dofbot_jointPoses()
            if (np.sum(np.isclose(np.array(current_joint_state), np.array(target_joint_state), atol = 1e-2)) == 5):
                current_state = MOVE_STATE
        elif current_state == MOVE_STATE:
            # 设置目标位置
            target_pos = (env.get_target_pose()[0] + obj_offset2[0], env.get_target_pose()[1] + obj_offset2[1], block_pos[2] + obj_offset2[2])
            # 计算目标位置对应的关节角度
            target_joint_state = env.dofbot_setInverseKine(target_pos)
            env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)
            # 检查是否到达目标位置
            current_joint_state, gripper_angle = env.get_dofbot_jointPoses()
            if (np.sum(np.isclose(np.array(current_joint_state), np.array(target_joint_state), atol = 1e-2)) == 5):
                current_state = BACK_STATE
        elif current_state == BACK_STATE:
            # 设定1s后自动释放物体
            if start_time is None:
                start_time = time.time()
            current_time = time.time()
            # 设置目标位置
            target_pos = env.get_target_pose()
            target_pos = (target_pos[0] + obj_offset3[0], target_pos[1] + obj_offset3[1], block_pos[2] + obj_offset3[2])
            # 计算目标位置对应的关节角度
            target_joint_state = env.dofbot_setInverseKine(target_pos)
            current_joint_state, _ = env.get_dofbot_jointPoses()
            # 1s自动释放
            if (current_time - start_time > 1.0):
                env.dofbot_control(target_joint_state, GRIPPER_DEFAULT_ANGLE)
            else:
                env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)


        Reward = env.reward()
