import roboticstoolbox as rtb
import numpy as np

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


target_pos = np.array([
    [-1., 0., 0., 0.1,],
    [0., 1., 0., 0.],
    [0., 0., -1., -0.1],
    [0., 0., 0., 1.]
])

# 输出机器人逆运动学demo
ikine_result = dofbot.ik_LM(target_pos)[0]
print("ikine: ", np.array(ikine_result) / 3.14 * 180.)


# 输出机器人正运动学demo
fkine_result = dofbot.fkine(ikine_result)
print("fkine: ", fkine_result)

# 展示机器人正运动学demo
dofbot.plot(q=ikine_result, block=True)