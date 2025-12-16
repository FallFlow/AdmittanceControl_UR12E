from fcntl import F_DUPFD

import numpy as np
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time
import filter_module
from scipy.spatial.transform import Rotation as R

# 创建RTDE接收接口实例
rtde_c = RTDEControl("192.168.0.100")
rtde_recv = RTDEReceive("192.168.0.100")  # 机器人IP

pi = np.pi

# 初始化
q_init = [pi/2, -pi/2, pi/2, -pi, -pi/2, 0]   # 初始化机器人位置
rtde_c.moveJ(q_init)    # 在关节空间下线性移动到该位置
time.sleep(1)   # 阻塞1秒使末端力传感器稳定

# while True:
#     print( rtde_recv.getActualTCPForce() )
#     time.sleep(1)

x_init = rtde_recv.getActualTCPPose()  # 当前TCP位姿就是笛卡儿空间下的初始位姿
"""
笛卡儿空间下的初始位姿也可以通过运动学正解得到
# x_init = rtde_c.getForwardKinematics(q_init)
这个值应当与rtde_recv.getActualTCPPose()得到的x_init相等
但是经过测试，getForwardKinematics()似乎有问题，无法得到正确的x_init
经过测试，getInverseKinematics()是正常的，使用
# print( getInverseKinematics(x_init) )
可以得到和q_init十分接近的值
"""

print(x_init)
x_d   = x_init               # 将期望位姿设置为初始位姿
dx_d  = [0, 0, 0, 0, 0, 0]   # 期望速度设置为0，即希望机械臂保持在初始位姿不动
ddx_d = [0, 0, 0, 0, 0, 0]
F_d   = [0, 0, 0, 0, 0, 0]

rtde_c.zeroFtSensor()   # 将末端六维力传感器置零
time.sleep(1)

"""
传感器零漂补偿
由于传感器零点漂移较严重，所以要进行零漂补偿
滤波器的相关函数见filter.py
"""
comp_filter = filter_module.MoveMeanFilter(window_size=15)   # 建立传感器力补偿的滤波器实例，使用滑动平均值滤波器，窗口长度15
for i in range(100):    # 每0.02秒取一次末端六维力数据，循环100次，将滤波结果存在F_ext_comp中
    F_ext_comp = comp_filter.update(rtde_recv.getActualTCPForce())
    time.sleep(0.02)
F_ext = rtde_recv.getActualTCPForce() - F_ext_comp  #将实测的外部力减去零漂补偿

"""
六维力传感器信号滤波
由于传感器噪声较严重，所以要进行信号滤波
滤波器的相关函数见Filter.py
"""
F_ext_filter = filter_module.RCLowPassFilter(alpha=0.35, initial_value=F_ext)  # 建立外部力低通滤波器实例

# 设计导纳控制参数
M = [0.8, 0.8, 0.8, 0.01, 0.01, 0.01]   # 惯性矩阵
D = [8, 8, 8, 0.006, 0.006, 0.005]      # 阻尼矩阵
K = [80, 80, 80, 1.1, 1.1, 2]           # 刚度矩阵
dt= 0.02                                # 离散时间，后面会用到
# 由于传感器噪声严重，设置死区
Deadband = [1.5, 1.5, 1.5, 0.015, 0.05, 0.3]

def admittance_control(M, D, K, dt, x, dx, F_ext):
    """导纳控制

    参数:
    M     -- 导纳惯性矩阵
    D     -- 导纳阻尼矩阵
    K     -- 导纳刚度矩阵
    dt    -- 离散时间
    x     -- 当前笛卡儿位姿
    dx    -- 当前速度
    F_ext -- 当前外部作用力

    返回:
    目标笛卡儿位姿
    """

    ddx_c = [0] * 6
    dx_c = [0] * 6
    deltax_c = [0] * 6
    x_c = [0] * 6

    e = [0] * 6
    de = [0] * 6
    e_F = [0] * 6

    e[0:3] = np.array(x_d[0:3]) - np.array(x[0:3])
    rot_x_d = R.from_rotvec(x_d[3:6])
    rot_x = R.from_rotvec(x[3:6])
    e[3:6] = (rot_x_d * rot_x.inv()).as_rotvec()  # 计算四元数误差

    de[0:6] = np.array(dx_d[0:6]) - np.array(dx[0:6])

    e_F[0:6] = np.array(F_d[0:6]) - np.array(F_ext[0:6])
    for i in range(6):  # 处理死区
        if abs(e_F[i]) < Deadband[i]:
            e_F[i] = 0

    for i in range(6):  # 导纳控制
        # 应用导纳控制公式：M*(ddx_d - ddx) + D*(dx_d - dx) + K*(x_d - x) = (F_d - F_ext)
        ddx_c[i] = ( D[i] * de[i] + K[i] * e[i] - e_F[i] ) / M[i] + ddx_d[i]
        dx_c[i] = dx[i] + ddx_c[i] * dt
        deltax_c[i] = dx_c[i] * dt

    x_c[0:3] = np.array(x[0:3]) + np.array(deltax_c[0:3])               # 位置可以直接做加法
    x_c[3:6] = ( R.from_rotvec(deltax_c[3:6]) * rot_x ).as_rotvec()    # 姿态需要四元数左乘
    return x_c #, dx_c, ddx_c

# 初始化力补偿滤波器实例，这次对每个方向的力都设计一个补偿器
comp = [filter_module.MoveMeanFilter(window_size=5),
        filter_module.MoveMeanFilter(window_size=5),
        filter_module.MoveMeanFilter(window_size=5),
        filter_module.MoveMeanFilter(window_size=5),
        filter_module.MoveMeanFilter(window_size=5),
        filter_module.MoveMeanFilter(window_size=5),]

def ExternalForceComp():
    """外部力补偿
    实时更新零漂补偿F_ext_comp
    """
    for i in range(6):
        if abs(F_ext[i]) < Deadband[i]:
            F_ext_comp[i] = comp[i].update(F_ext[i])
    return


print('Start')
while True:
    t_start = rtde_c.initPeriod()   # 和结尾的rtde_c.waitPeriod(t_start)一起，控制while循环时间为dt

    x = rtde_recv.getActualTCPPose()    # 获取当前TCP位姿
    dx = rtde_recv.getActualTCPSpeed()  # 获取当前TCP速度
    F_ext = F_ext_filter.update( rtde_recv.getActualTCPForce() ) - F_ext_comp   # 获取当前外部力
    ExternalForceComp() # 实时更新外部力补偿
    # print('Comp', F_ext_comp)

    x_c = admittance_control(M, D, K, dt, x, dx, F_ext)  # 使用导纳控制器得到柔顺位姿
    # print('F_ext', F_ext)
    print('                                                                                            ', end='\r')
    print('x_c', [round(x, 3) for x in x_c], end='')
    rtde_c.servoL(x_c, 0, 0, dt, 0.03, 100) # 在笛卡儿空间中伺服到柔顺位姿
    # 也可以逆解得到关节空间的柔顺角度，再用servoJ伺服
    # q_c = rtde_c.getInverseKinematics(x_c)
    # rtde_c.servoJ(q_c, 0, 0, dt, 0.03, 100)

    rtde_c.waitPeriod(t_start)      # 和开头的t_start = rtde_c.initPeriod()一起，控制while循环时间为dt