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

    # 用于存储读取的数据
datas_time_list = []
datas_position_x = []
datas_position_y = []
datas_position_z = []
datas_vel_forward = []
datas_vel_z = []

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

        # 初始化计数器
        count = 0

        # 逐行读取数据
        for row in reader:
            # 每隔十行取一次数据
            if count % 10 == 0:
                # 将每行数据转换为相应的数据类型并存储到相应的列表中
                time_list.append(float(row[0]))
                position_x.append(float(row[1]))
                position_y.append(float(row[2]))
                position_z.append(float(row[3]))
                vel_forward.append(float(row[4]))
                vel_z.append(float(row[5]))
            # 更新计数器
            count += 1

    start_time=time_list[0]
    for k in range(len(time_list)):
        time_list[k]=time_list[k]-start_time
        # if k<20:
        #     print("time",k," :",time_list[k])

    print("len(time_list):",len(time_list))
    print("len(position_x):",len(position_x))

    # print("time[0]:",time_list[0])
    # print("time[1]:",time_list[1])
    # print("time[2]:",time_list[2])


    datas_time_list.append(time_list)
    datas_position_x.append(position_x)
    datas_position_y.append(position_y)
    datas_position_z.append(position_z)
    datas_vel_forward.append(vel_forward)
    datas_vel_z.append(vel_z)

    # 计算位置和速度的方差
        
    position_x_std=[]
    position_y_std=[]
    position_z_std=[]
    vel_forward_std=[]
    vel_z_std=[]
        
        
    position_x_mean=[]
    position_y_mean=[]
    position_z_mean=[]
    vel_forward_mean=[]
    vel_z_mean=[]
        
longest_time_list = max(datas_time_list, key=len)
print("The longest list is:", len(longest_time_list))


for k in range(len(longest_time_list)):

    compare_data_position_x=[]
    for position_x in datas_position_x:
        if k > len(position_x)-1:
            position_x_k_=position_x[-1]
        else:
            position_x_k_=position_x[k]
        compare_data_position_x.append(position_x_k_)

    position_x_std.append(np.std(compare_data_position_x))
    position_x_mean.append(np.mean(compare_data_position_x))

    compare_data_position_y=[]
    for position_y in datas_position_y:
        if k > len(position_y)-1:
            position_y_k_=position_y[-1]
        else:
            position_y_k_=position_y[k]
        compare_data_position_y.append(position_y_k_)

    position_y_std.append(np.std(compare_data_position_y))
    position_y_mean.append(np.mean(compare_data_position_y))

    compare_data_position_z=[]
    for position_z in datas_position_z:
        if k > len(position_z)-1:
            position_z_k_=position_z[-1]
        else:
            position_z_k_=position_z[k]
        compare_data_position_z.append(position_z_k_)

    position_z_std.append(np.std(compare_data_position_z))
    position_z_mean.append(np.mean(compare_data_position_z))

    compare_data_vel_forward=[]
    for vel_forward in datas_vel_forward:
        if k > len(vel_forward)-1:
            vel_forward_k_=vel_forward[-1]
        else:
            vel_forward_k_=vel_forward[k]
        compare_data_vel_forward.append(vel_forward_k_)

    vel_forward_std.append(np.std(compare_data_vel_forward))
    vel_forward_mean.append(np.mean(compare_data_vel_forward))

    compare_data_vel_z=[]
    for vel_z in datas_vel_z:
        if k > len(vel_z)-1:
            vel_z_k_=vel_z[-1]
        else:
            vel_z_k_=vel_z[k]
        compare_data_vel_z.append(vel_z_k_)

    vel_z_std.append(np.std(compare_data_vel_z))
    vel_z_mean.append(np.mean(compare_data_vel_z))


# 绘制带方差的折线图
plt.figure(figsize=(10, 5))

plt.plot(longest_time_list, position_x_mean, label='X Position')
plt.fill_between(longest_time_list, np.array(position_x_mean) - position_x_std, np.array(position_x_mean) + position_x_std, alpha=0.3)

plt.plot(longest_time_list, position_y_mean, label='Y Position')
plt.fill_between(longest_time_list, np.array(position_y_mean) - position_y_std, np.array(position_y_mean) + position_y_std, alpha=0.3)

plt.plot(longest_time_list, position_z_mean, label='Z Position')
plt.fill_between(longest_time_list, np.array(position_z_mean) - position_z_std, np.array(position_z_mean) + position_z_std, alpha=0.3)

plt.xlabel('Time (s)',fontsize=15)
plt.ylabel('Position (m)',fontsize=15)
plt.legend(fontsize=12,loc='lower right')
plt.savefig(os.path.join('plots', file_name.split('.')[0] + '_position.pdf'),tight_layout='tight', bbox_inches='tight')
plt.show()

plt.figure(figsize=(10, 5))

plt.plot(longest_time_list, vel_forward_mean, label='Forward Velocity')
plt.fill_between(longest_time_list, np.array(vel_forward_mean) - vel_forward_std, np.array(vel_forward_mean) + vel_forward_std, alpha=0.3)

plt.plot(longest_time_list, vel_z_mean, label='Z-axis Velocity')
plt.fill_between(longest_time_list, np.array(vel_z_mean) - vel_z_std, np.array(vel_z_mean) + vel_z_std, alpha=0.3)

plt.xlabel('Time (s)',fontsize=15)
plt.ylabel('Velocity (m/s)',fontsize=15)
plt.legend(fontsize=12,loc='upper right')
plt.savefig(os.path.join('plots', file_name.split('.')[0] + '_velocity.pdf'),tight_layout='tight', bbox_inches='tight')
plt.show()
