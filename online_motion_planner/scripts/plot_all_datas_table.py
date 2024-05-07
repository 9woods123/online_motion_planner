import os
import csv
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42


# 设置数据文件夹路径
data_folder = 'data_terrain/'

# 获取数据文件夹下所有CSV文件名
file_names = [file for file in os.listdir(data_folder) if file.endswith('.csv')]

# 存储多组数据的全程轨迹长度和最大速度
total_trajectory_lengths = []
max_velocities = []
max_z_velocities = []

# 遍历每个CSV文件并绘制带方差的折线图
for file_name in file_names:
    # 用于存储读取的数据
    time_list = []
    position_x = []
    position_y = []
    position_z = []
    vel_forward = []
    vel_z = []

    # 打开CSV文件并读取数据
    with open(os.path.join(data_folder, file_name), 'r', newline='') as csvfile:
        reader = csv.reader(csvfile)

        # 读取表头
        headers = next(reader)

        # 逐行读取数据
        for row in reader:
            # 将每行数据转换为相应的数据类型并存储到相应的列表中
            time_list.append(float(row[0]))
            position_x.append(float(row[1]))
            position_y.append(float(row[2]))
            position_z.append(float(row[3]))
            vel_forward.append(float(row[4]))
            vel_z.append(float(row[5]))

    # 计算时间间隔
    delta_t = np.diff(time_list)

    # 计算每个时间间隔内的位移
    delta_x = np.diff(position_x)
    delta_y = np.diff(position_y)
    delta_z = np.diff(position_z)

    # 计算每个时间间隔内的位移的平方和
    delta_squared_sum = delta_x**2 + delta_y**2 + delta_z**2

    # 计算全程轨迹长度
    total_trajectory_length = np.sum(np.sqrt(delta_squared_sum))

    # 计算最大速度
    max_velocity = np.max(vel_forward)
    max_z_velocity = np.max(vel_z)

    # 存储轨迹长度和最大速度
    total_trajectory_lengths.append(total_trajectory_length)
    max_velocities.append(max_velocity)
    max_z_velocities.append(max_z_velocity)


print("Total Trajectory Length Max:",  np.max(total_trajectory_lengths))
print("Total Trajectory Length Mean:",  np.mean(total_trajectory_lengths))
print("Total Trajectory Length Std:",  np.std(total_trajectory_lengths))

print("Max Velocity Max:", np.max(max_velocities))
print("Max Velocity Mean", np.mean(max_velocities))
print("Max Velocity Std", np.std(max_velocities))

print("Max Z Velocity Max:", np.max(max_z_velocities))
print("Max Z Velocity Mean", np.mean(max_z_velocities))
print("Max Z Velocity Std", np.std(max_z_velocities))



# 打印每组数据的全程轨迹长度和最大速度
for i, file_name in enumerate(file_names):
    print("File:", file_name)
    print("Total Trajectory Length:", total_trajectory_lengths[i])
    print("Max Velocity:", max_velocities[i])
    print("Max Z Velocity:", max_z_velocities[i])
    print()

