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
# 用改进DH参数发表示机器人正运动学
dofbot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(a = 1, alpha=11, d=111, offset=1111),
        rtb.RevoluteMDH(),
        rtb.RevoluteMDH(),
        rtb.RevoluteMDH(),
        rtb.RevoluteMDH()
    ]
)

# 用改进DH参数发表示机器人正运动学
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
    [-1., 0., 0., 0.1,],
    [0., 1., 0., 0.],
    [0., 0., -1., -0.1],
    [0., 0., 0., 1.]
])
ikine_result0 = dofbot.ik_LM(target_pos0)[0]
print("ikine: ", np.array(ikine_result0))
dofbot.plot(q=ikine_result0, block=True)


#part2-1
target_pos1 = np.array([
    [1., 0., 0., 0.1,],
    [0., 1., 0., 0.],
    [0., 0., 1., 0.1],
    [0., 0., 0., 1.]
])
ikine_result1 = dofbot.ik_LM(target_pos1)[0]
print("ikine: ", np.array(ikine_result1))
dofbot.plot(q=ikine_result1, block=True)


#part2-2
target_pos2 = np.array([
    [math.cos(-pi/3), 0., -math.sin(-pi/3), 0.2,],
    [0., 1., 0., 0.],
    [math.sin(-pi/3), 0., math.cos(-pi/3), 0.2],
    [0., 0., 0., 1.]
])
ikine_result2 = dofbot.ik_LM(target_pos2)[0]
print("ikine: ", np.array(ikine_result2))
dofbot.plot(q=ikine_result2, block=True)


#part2-3
target_pos3 = np.array([
    [-0.866, -0.25, -0.433, -0.03704,],
    [0.5, -0.433, -0.75, -0.06415],
    [0., -0.866, 0.5, 0.3073],
    [0., 0., 0., 1.]
])
ikine_result3 = dofbot.ik_LM(target_pos3)[0]
print("ikine: ", np.array(ikine_result3))
dofbot.plot(q=ikine_result3, block=True)


pos_que_x = []
pos_que_y = []
pos_que_z = []
for J0 in range(-180, 181, 360 // 20):
    for J1 in range(-90, 91, 180 // 5):
        for J2 in range(-150, 151, 300 // 5):
            for J3 in range(-100, 101, 200 // 5):
                q = [J0, J1, J2, J3, 0]
                fkine = dofbot.fkine(q)
                pos = [fkine.A[0][3], fkine.A[1][3], fkine.A[2][3]]
                pos_que_x.append(pos[0])
                pos_que_y.append(pos[1])
                pos_que_z.append(pos[2])

fig = plt.figure()
ax = plt.axes(projection ="3d")
ax.scatter(pos_que_x, pos_que_y, pos_que_z)
plt.show()