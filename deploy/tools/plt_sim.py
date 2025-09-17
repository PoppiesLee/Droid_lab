import pickle
import numpy as np
import matplotlib.pyplot as plt
import os
from deploy.tools.data_var import Q
script_dir = os.path.dirname(os.path.abspath(__file__))
# 过滤出文件名，并按名称排序
data_dir = script_dir + '/data/'
files = os.listdir(data_dir)
files = sorted(f for f in files)
# 获取最后的文件名
data_name = data_dir + files[-1]
# data_name = data_dir + 'data_2024-11-28_14-30-34.pickle'
print(data_name)

# 从文件读取字典
f = open(data_name, 'rb')
loaded_data = pickle.load(f)
Q    = loaded_data['Q']
data = loaded_data['data']
QKey = Q.keys()
QKey_list = list(QKey)

# 创建绘图窗口和子图
num_windows = 0
for key in QKey_list:
    num_windows = max(num_windows, Q[key][0]) # 根据字典中的窗口数量设置子图数量
Nc = np.ceil(np.sqrt(num_windows)).astype(int)  #根据Q需要，设置subPlot的行列数
Nr = np.ceil(num_windows/Nc).astype(int)
fig, axs = plt.subplots(Nr, Nc, figsize=(5 * Nr, 5 * Nc))
if num_windows == 1:
    axs = [axs]  # 确保axs始终是列表形式，方便后续处理

# 开始绘图
for i in range(1, len(Q)):
    ax_id = Q[QKey_list[i]][0]
    if 1 <= ax_id <= Nc*Nr:
        ax_row = np.ceil(ax_id / Nc).astype(int) - 1
        ax_col = ax_id - ax_row * Nc - 1
        x = data[QKey_list[0]]
        y = data[QKey_list[i]] * Q[QKey_list[i]][3]
        line,=axs[ax_row][ax_col].plot(x,y,
                                       label=QKey_list[i],
                                       linestyle=Q[QKey_list[i]][1],
                                       color=Q[QKey_list[i]][2])
        axs[ax_row][ax_col].legend(prop={'size': 14})  # 设置图例字体大小为15)
        axs[ax_row][ax_col].tick_params(labelsize=14)
        axs[ax_row][ax_col].grid(True)
# plt.tight_layout()
plt.suptitle(data_name, size=20)
plt.show()
plt.ion()