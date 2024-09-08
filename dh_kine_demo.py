import roboticstoolbox as rtb

pi = 3.1415926          # 定义pi常数

l1 = 0.1045             # 定义第一连杆长度
l2 = 0.08285            # 定义第三连杆长度
l3 = 0.08285            # 定义第四连杆长度
l4 = 0.12842            # 定义第五连杆长度

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

# 输出机器人正运动学demo
print(dofbot.fkine([0., 20. / 180. * pi, 15. / 180. * pi, 10. / 180. * pi, 0.]))

# 展示机器人正运动学demo
dofbot.plot(q=[0., 40. / 180. * pi, 30. / 180. * pi, 20. / 180. * pi, 0.], block=True)