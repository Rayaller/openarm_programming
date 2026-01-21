#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import transforms3d

# 导入我们在同级目录下创建的类
from hello_moveit_py.moveit_client import MoveItActionClient

def main():
    rclpy.init()
    node = rclpy.create_node("hello_moveit_py")

    # 1. 实例化我们自己写的客户端
    # 请确认你的 group_name (如 "left_arm") 
    # 和末端 link 名称 (在 moveit_client.py 里修改 link_name，如果是 openarm 通常可能是 link6 或 gripper_base)
    moveit = MoveItActionClient(node, group_name="left_arm", base_frame="base_link")
    # moveit = MoveItActionClient(node, group_name="left_arm", base_frame="world")

    # 2. 设置目标
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"
    
    # 位置
    target_pose.pose.position.x = 0.139
    target_pose.pose.position.y = 0.109
    target_pose.pose.position.z = 0.314

    # 姿态 (RPY -> Quaternion)
    w, x, y, z = transforms3d.euler.euler2quat(-3.046, 0.175, 0.044, 'sxyz')
    target_pose.pose.orientation.x = x
    target_pose.pose.orientation.y = y
    target_pose.pose.orientation.z = z
    target_pose.pose.orientation.w = w

    # 3. 执行
    node.get_logger().info("开始运动...")
    moveit.move_to_pose(target_pose)

    rclpy.shutdown()

if __name__ == "__main__":
    main()