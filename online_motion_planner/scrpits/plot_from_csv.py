import csv
import numpy as np
import matplotlib.pyplot as plt
import math

# 指定要读取的CSV文件名
filename = 'data.csv'

# 用于存储读取的数据
time_list = []
position_x = []
position_y = []
position_z = []
vel_forward = []
vel_z = []

# 打开CSV文件并读取数据
with open(filename, 'r', newline='') as csvfile:
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

# 打印读取的数据
print("Time:", time_list)
print("Position X:", position_x)
print("Position Y:", position_y)
print("Position Z:", position_z)
print("Velocity Forward:", vel_forward)
print("Velocity Z:", vel_z)

plt.figure(figsize=(10, 5))


plt.plot(time_list, position_x, label='X  '+' Position')

plt.plot(time_list, position_y, label='Y  '+' Position')

plt.plot(time_list, position_z, label='Z  '+' Position')

plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.legend()
plt.savefig('position.pdf')
plt.show()


plt.figure(figsize=(10, 5))

plt.plot(time_list, vel_forward, label='Forward  '+' Velocity')
plt.plot(time_list, vel_z, label='Z-axis '+' Velocity')


plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.legend()
plt.savefig('velocity.pdf')
plt.show()