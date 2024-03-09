#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import time
from gazebo_msgs.msg import ModelStates
import numpy as np
import matplotlib.pyplot as plt
import math
import csv

class experiment:
        def __init__(self):
            self.set_goal_pub= rospy.Publisher("/goal", PoseStamped, queue_size=10)  

        def experiment_start(self):
            print("experiment_start")
            
            goal=PoseStamped()
            goal.header.frame_id="world"
            goal.header.stamp=rospy.Time.now()
            goal.header.seq=1

            goal.pose.position.x=106.553
            goal.pose.position.y=-72.799
            goal.pose.position.z=-85.000
            goal.pose.orientation.x=0.000
            goal.pose.orientation.y=0.000
            goal.pose.orientation.z= -0.235
            goal.pose.orientation.w=0.972
            print(goal)
            self.set_goal_pub.publish(goal)


class PoseRecorder:
    def __init__(self):
        self.path = Path()
        self.path.header.frame_id = "world"  # 设置路径的固定帧
        self.path_pub = rospy.Publisher("/recorded_path", Path, queue_size=10)  # 发布路径的主题

        self.last_record_time = rospy.Time.now()
        
        self.model_state = []
        self.model_vel_state = []

        self.time = []

        rospy.Subscriber("/eca_a9/ground_truth_to_tf_eca_a9/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.modelState_callback)


    def pose_callback(self, pose_msg):
        current_time = rospy.Time.now()
        if (current_time - self.last_record_time).to_sec() >= 2.0:  # 每1秒记录一个pose
            self.path.poses.append(pose_msg)
            self.path.header.stamp = rospy.Time.now()  # 更新路径的时间戳
            self.path_pub.publish(self.path)  # 发布路径
            self.last_record_time = current_time


    def modelState_callback(self, state_msg):
        current_time = rospy.get_time()
        self.time.append(current_time)
        
        ##  记录pose数据

        auv_pose = state_msg.pose[state_msg.name.index("eca_a9")]
        self.model_state.append(np.array([auv_pose.position.x,
                                          auv_pose.position.y,
                                          auv_pose.position.z]))
        
        ##  记录linear velocity数据
        auv_vel= state_msg.twist[state_msg.name.index("eca_a9")]
        self.model_vel_state.append(np.array([auv_vel.linear.x,
                                          auv_vel.linear.y,
                                          auv_vel.linear.z]))
        


    def plot_data(self):
        
        min_length = min(len(self.time), len(self.model_state),len(self.model_vel_state))
        time_list = self.time[:min_length]

        position_x = [pose[0] for pose in self.model_state][:min_length]

        position_y = [pose[1] for pose in self.model_state][:min_length]

        position_z = [pose[2] for pose in self.model_state][:min_length]

        vel_forward=[math.sqrt(self.model_vel_state[k][0]**2+self.model_vel_state[k][1]**2+self.model_vel_state[k][2]**2)
                       for k in range(0,min_length)]
        
        vel_z = [vel[2] for vel in self.model_vel_state][:min_length]

        # 将数据组合成一个列表，每个元素是一行数据
        data = zip(time_list,position_x,position_y,position_z, vel_forward,vel_z)

        # 指定要写入的CSV文件名
        filename = 'data.csv'

        # 将数据写入CSV文件
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # 写入表头
            writer.writerow(['time', 'position_x', 'position_y','position_z','vel_forward','vel_z'])
            
            # 逐行写入数据
            for row in data:
                writer.writerow(row)

        print("Data has been saved to", filename)


def main():
    rospy.init_node('pose_recorder_node')
    pose_recorder = PoseRecorder()

    exp1=experiment()
    rate = rospy.Rate(1)  # 设置循环频率为1Hz，即每秒一次循环

    count = 0
    while not rospy.is_shutdown():
        exp1.experiment_start()
        rate.sleep()  # 在每次循环迭代之间添加1秒的延迟
        count += 1
        if count >= 10:  # 如果循环迭代次数达到1次，退出循环
            break


    rospy.on_shutdown(pose_recorder.plot_data)
    rospy.spin()

if __name__ == '__main__':
    main()
