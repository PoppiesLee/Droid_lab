import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV文件（不自动分配列名）
data = pd.read_csv('joint_positions_20250703-154156.csv', header=None)

# 提取前14列（目标位置）和后14列（当前位置）
target_positions = data.iloc[:, :14]
current_positions = data.iloc[:, 14:28]

# 创建时间轴（假设每行数据间隔相同）
time = range(len(data))

# 设置图形大小
plt.figure(figsize=(15, 10))

# 绘制目标位置
plt.subplot(2, 1, 1)
for i in range(14):
    plt.plot(time, target_positions.iloc[:, i], label=f'Joint {i+1}')
plt.title('Target Joint Positions')
plt.xlabel('Time')
plt.ylabel('Position')
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid(True)

# 绘制当前位置
plt.subplot(2, 1, 2)
for i in range(14):
    plt.plot(time, current_positions.iloc[:, i], label=f'Joint {i+1}')
plt.title('Current Joint Positions')
plt.xlabel('Time')
plt.ylabel('Position')
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid(True)

# 调整布局
plt.tight_layout()
plt.show()