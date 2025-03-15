import time
import numpy as np
# 创建机械臂对象
import rospy
from dofbot_real import RealEnv


def linear_interpolation(src, tat, n=10):
    """简单的线性插值实现"""
    path = np.linspace(src, tat, num=n)
    return path

if __name__ == '__main__':
    # 调用realenv
    env = RealEnv()
    env.reset()


    # 可以实现简单状态机来实现分段控制
    # 状态机的中间路点
    points = [
        [np.asarray([90., 90., 90., 90., 90.],dtype=float),90], # 初始位置
        [np.asarray([132., 51., 42., 9., 91.],dtype=float),148], 
        [np.asarray([132., 51., 70., 9., 91.],dtype=float),148],
        [np.asarray([41., 53., 70., 9., 91.],dtype=float),148], 
        [np.asarray([41., 53., 44., 9., 91.],dtype=float),80],  
    ]

    for i in range(len(points) - 1):
        # 取出路点并做路径规划得到路径
        path = linear_interpolation(points[i][0], points[i+1][0], n=20)
        grippers = linear_interpolation(points[i][1], points[i+1][1], n=5)
        for p in path:
            # 执行路径上各点
            # env.step(joint=...)可以控制关节
            # env.step(gripper=...)可以控制夹爪
            # 建议分开控制
            env.step(joint=p)
            print("STATE",env.get_state())
            print("G",p)
        if points[i][1] != points[i+1][1]:
            for g in grippers:
                env.step(gripper=g)
                print("STATE",env.get_state())
                print("G",g)

