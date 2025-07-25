from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time

# 创建RTDE接收接口实例

rtde_c = RTDEControl("192.168.0.100")
rtde_recv = RTDEReceive("192.168.0.100")  # 机器人IP

pi = 3.1415926

q_init  = [pi/2, -pi/2, pi/2, -pi/2, pi/2, 0]
rtde_c.moveJ(q_init)
time.sleep(1)

rtde_c.zeroFtSensor()
time.sleep(1)

while True:
    F_ext = rtde_recv.getActualTCPForce()
    print('F_ext',F_ext)
    time.sleep(0.5)
