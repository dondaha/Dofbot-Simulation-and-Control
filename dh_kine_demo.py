import roboticstoolbox as rtb
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

pi = 3.1415926          # 定义pi常数

l1 = 0.1045             # 定义第一连杆长度
l2 = 0.08285            # 定义第三连杆长度
l3 = 0.08285            # 定义第四连杆长度
l4 = 0.12842            # 定义第五连杆长度

# student version
# Problem1. 用改进DH参数发表示机器人正运动学
dofbot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=l1),
        rtb.RevoluteMDH(alpha=-pi / 2, offset=-pi / 2),
        rtb.RevoluteMDH(a=l2),
        rtb.RevoluteMDH(a=l3, offset=pi / 2),
        rtb.RevoluteMDH(alpha=pi / 2, d=l4)
    ]
)

# 输出机器人DH参数矩阵
print(dofbot)


'''
Part1 给出一下关节姿态时的机械臂正运动学解，并附上仿真结果
0.(demo) [0., pi/3, pi/4, pi/5, 0.]
1.[pi/2, pi/5, pi/5, pi/5, pi]
2.[pi/3, pi/4, -pi/3, -pi/4, pi/2]
3.[-pi/2, pi/3, -pi/3*2, pi/3, pi/3]
'''

# part1 demo
fkine_input0 = [0., pi/3, pi/4, pi/5, 0.]
fkine_result0 = dofbot.fkine(fkine_input0)
print(fkine_result0)
dofbot.plot(q=fkine_input0, block=True)

# part1-1
fkine_input1 = [pi/2, pi/5, pi/5, pi/5, pi]
fkine_result1 = dofbot.fkine(fkine_input1)
print(fkine_result1)
dofbot.plot(q=fkine_input1, block=True)

# part1-2
fkine_input2 = [pi/3, pi/4, -pi/3, -pi/4, pi/2]
fkine_result2 = dofbot.fkine(fkine_input2)
print(fkine_result2)
dofbot.plot(q=fkine_input2, block=True)

# part1-3
fkine_input3 = [-pi/2, pi/3, -pi/3*2, pi/3, pi/3]
fkine_result3 = dofbot.fkine(fkine_input3)
print(fkine_result3)
dofbot.plot(q=fkine_input3, block=True)


'''
Part1 给出一下关节姿态时的机械臂逆运动学解，并附上仿真结果
0.(demo) 
    [
        [-1., 0., 0., 0.1,],
        [0., 1., 0., 0.],
        [0., 0., -1., -0.1],
        [0., 0., 0., 1.]
    ]
1.
    [
        [1., 0., 0., 0.1,],
        [0., 1., 0., 0.],
        [0., 0., 1., 0.1],
        [0., 0., 0., 1.]
    ]
2.
    [
        [cos(pi/3), 0., -sin(pi/3), 0.05,],
        [0., 1., 0., 0.03],
        [sin(pi/3), 0., cos(pi/3)., -0.1],
        [0., 0., 0., 1.]
    ]
3.
    [
        [-0.866, -0.25, -0.433, -0.03704,],
        [0.5, -0.433, -0.75, -0.06415],
        [0., -0.866, 0.5, 0.3073],
        [0., 0., 0., 1.]
    ]
'''

#part2 demo
target_pos0 = np.array([
    [-1.0, 0.0, 0.0, 0.1,],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0, -0.1],
    [0.0, 0.0, 0.0, 1.0]
])
ikine_result0 = dofbot.ik_LM(target_pos0)[0]
print("ikine: ", np.array(ikine_result0))
dofbot.plot(q=ikine_result0, block=True)


#part2-1
target_pos1 = np.array([
    [1.0, 0.0, 0.0, 0.1,],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.1],
    [0.0, 0.0, 0.0, 1.0]
])
ikine_result1 = dofbot.ik_LM(target_pos1)[0]
print("ikine: ", np.array(ikine_result1))
dofbot.plot(q=ikine_result1, block=True)


#part2-2
target_pos2 = np.array([
    [math.cos(pi/3), 0.0, -math.sin(-pi/3), 0.2,],
    [0.0, 1.0, 0.0, 0.0],
    [math.sin(pi/3), 0.0, math.cos(-pi/3), 0.2],
    [0.0, 0.0, 0.0, 1.0]
])
ikine_result2 = dofbot.ik_LM(target_pos2)[0]
print("ikine: ", np.array(ikine_result2))
dofbot.plot(q=ikine_result2, block=True)


#part2-3
target_pos3 = np.array([
    [-0.866, -0.25, -0.433, -0.03704,],
    [0.5, -0.433, -0.75, -0.06415],
    [0.0, -0.866, 0.5, 0.3073],
    [0.0, 0.0, 0.0, 1.0]
])
ikine_result3 = dofbot.ik_LM(target_pos3)[0]
print("ikine: ", np.array(ikine_result3))
dofbot.plot(q=ikine_result3, block=True)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 定义关节角度限位
joint_limits = [
    np.deg2rad([-180, 180]),  # J1
    np.deg2rad([-90, 90]),    # J2
    np.deg2rad([-150, 150]),  # J3
    np.deg2rad([-100, 100]),  # J4
    np.deg2rad([-180, 180])   # J5
]

# 采样点数
num_samples = 50000

# 生成随机关节角度
joint_samples = np.random.uniform(
    low=[limit[0] for limit in joint_limits],
    high=[limit[1] for limit in joint_limits],
    size=(num_samples, len(joint_limits))
)

# 计算每个采样点的末端执行器位置
workspace_points = []
for joint_angles in joint_samples:
    fkine_result = dofbot.fkine(joint_angles)
    position = fkine_result.t
    workspace_points.append(position)

workspace_points = np.array(workspace_points)

# 绘制工作空间
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(workspace_points[:, 0], workspace_points[:, 1], workspace_points[:, 2], c='r', marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Dofbot Workspace')
plt.show()