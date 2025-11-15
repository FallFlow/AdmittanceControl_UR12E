import numpy as np
from collections import deque
import time


class MoveMeanFilter:
    """滑动平均值滤波器"""

    def __init__(self, window_size=5):
        """滑动平均值滤波器初始化

        参数:
        window_size -- 滤波窗口长度 (默认为5)
        """
        self.window_size = window_size
        # 使用双端队列存储历史数据
        self.history = deque(maxlen=window_size)
        # 用于跟踪是否窗口已满
        self.window_full = False

    def update(self, current_input):
        """更新滤波器状态并返回滤波后的值

        参数:
        current_input -- 当前传感器数据（列表或数组）

        返回:
        平均值滤波后的数据
        """
        # 确保输入是numpy数组
        current_input = np.array(current_input)
        # 添加到历史记录
        self.history.append(current_input)
        # 检查窗口是否已满
        if not self.window_full and len(self.history) == self.window_size:
            self.window_full = True
        # 计算当前窗口的平均值，如果窗口不满，使用所有可用数据
        window_data = np.array(self.history)
        return np.mean(window_data, axis=0)

    def reset(self):
        """重置滤波器状态"""
        self.history.clear()
        self.window_full = False


def simulate_sensor():
    """模拟实时传感器数据生成"""
    # 基础值加上随机噪声
    base_value = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    noise = np.random.normal(0, 0.2, 6)  # 均值为0，标准差为0.2的高斯噪声
    return (np.array(base_value) + noise).tolist()


class RCLowPassFilter:
    """RC低通滤波器"""

    def __init__(self, alpha=0.5, initial_value=None):
        """RC低通滤波器初始化

        参数:
        alpha         -- 滤波系数 (0 < alpha < 1)，值越小滤波效果越强(平滑但延迟大)
        initial_value -- 初始值，应为6元素数组，默认为零向量
        """
        self.alpha = alpha
        self.last_output = np.zeros(6) if initial_value is None else np.array(initial_value)

    def update(self, current_input):
        """
        更新滤波器状态并返回滤波后的值

        参数:
        current_input -- 当前传感器读数，应为6元素数组

        返回:
        滤波后的6元素数组
        """
        # 应用滤波公式: y(k) = (1 - alpha) * y(k-1) + alpha * x(k)
        filtered = (1 - self.alpha) * self.last_output + self.alpha * np.array(current_input)

        # 保存当前输出作为下一次的上一次输出
        self.last_output = filtered.copy()

        return filtered



# 主循环示例
if __name__ == "__main__":
    # 创建滤波器实例，窗口大小为5
    filter = MoveMeanFilter(window_size=5)

    print("开始实时滤波处理...")
    print("按Ctrl+C停止")

    try:
        # 模拟实时数据流
        for i in range(20):
            # 模拟从传感器获取数据（每次获取一个六维点）
            raw_data = simulate_sensor()

            # 更新滤波器并获取平滑后的数据
            smoothed_data = filter.update(raw_data)

            # 打印结果
            print(f"\n时间步 {i + 1}:")
            print(f"原始数据: {np.round(raw_data, 4)}")
            print(f"滤波数据: {np.round(smoothed_data, 4)}")
            print(f"当前窗口大小: {len(filter.history)}/{filter.window_size}")

            # 模拟实时延迟
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n处理已停止")