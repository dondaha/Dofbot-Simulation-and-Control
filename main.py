

from dofbot import DofbotEnv
import numpy as np
import time



if __name__ == '__main__':
    env = DofbotEnv()
    env.reset()
    Reward = False
    while not Reward:

        jointPoses = [1.57, 0., 1.57, 1.57, 1.57]#np.zeros(5)
        gripperAngle = 0
        '''
        #获取物块位姿、目标位置和机械臂位姿，计算机器臂关节和夹爪角度，使得机械臂夹取绿色物块，放置到紫色区域。
        '''
        print(env.dofbot_forwardKine(jointPoses))
        # env.dofbot_control(jointPoses,gripperAngle)
        Reward = env.reward()

