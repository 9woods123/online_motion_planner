import csv
import numpy as np
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
# 打开CSV文件
path_comp_time=[]

with open('path_planning_comptime.csv', newline='') as csvfile:
    # 创建CSV读取器
    reader = csv.reader(csvfile)
    # 遍历每一行并输出
    for row in reader:

        path_comp_time.append(float(row[0]))

# print(path_comp_time)
print("computation time Max: ", np.max(path_comp_time))
print("computation time Mean: ", np.mean(path_comp_time))
print("computation time Std: ", np.std(path_comp_time))