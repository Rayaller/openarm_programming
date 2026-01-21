import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MotionPlanRequest, PositionConstraint, OrientationConstraint, JointConstraint, Constraints
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
import threading
import time

class MoveItActionClient:
    def __init__(self, node: Node, group_name="left_arm", base_frame="base_link", external_spin=False):
        self.node = node
        self.group_name = group_name
        self.base_frame = base_frame
        self.external_spin = external_spin
        
        self._cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(node, MoveGroup, 'move_action', callback_group=self._cb_group)
        
        self.node.get_logger().info(f"等待 MoveGroup ({group_name})...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f"MoveGroup {group_name} 未上线！")
        else:
            self.node.get_logger().info(f"MoveGroup {group_name} 已连接")

    def _wait_for_future(self, future, timeout=10.0):
        if self.external_spin:
            start_time = time.time()
            while not future.done():
                time.sleep(0.01)
                if time.time() - start_time > timeout:
                    self.node.get_logger().error("等待 Action 响应超时！")
                    return None
            return future.result()
        else:
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            return future.result()

    def _send_request(self, constraints, wait):
        """内部通用发送函数"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 1.0
        goal_msg.request.max_acceleration_scaling_factor = 1.0
        goal_msg.request.goal_constraints.append(constraints)

        self.node.get_logger().info(f"[{self.group_name}] 发送规划请求...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        goal_handle = self._wait_for_future(send_goal_future)
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error(f"[{self.group_name}] 请求被拒绝或超时")
            return False

        if wait:
            get_result_future = goal_handle.get_result_async()
            result_wrapper = self._wait_for_future(get_result_future, timeout=15.0)
            
            if result_wrapper and result_wrapper.result.error_code.val == 1:
                self.node.get_logger().info(f"[{self.group_name}] 执行成功!")
                return True
            else:
                err = result_wrapper.result.error_code.val if result_wrapper else "Unknown"
                self.node.get_logger().error(f"[{self.group_name}] 执行失败, 错误码: {err}")
                return False
        return True

    def move_to_pose(self, target_pose: PoseStamped, link_name="openarm_left_link7", wait=True):
        """笛卡尔空间规划 (Pose)"""
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.base_frame
        pos_constraint.link_name = link_name
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.001]
        pos_constraint.constraint_region.primitives = [s]
        pos_constraint.constraint_region.primitive_poses = [target_pose.pose]
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.base_frame
        ori_constraint.link_name = link_name
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.01
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        
        return self._send_request(constraints, wait)

    def move_to_joint(self, joint_values: dict, wait=True):
        """关节空间规划 (Joint) - 用于夹爪或机械臂归位"""
        constraints = Constraints()
        
        for joint_name, target_value in joint_values.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(target_value)
            jc.tolerance_above = 0.0002
            jc.tolerance_below = 0.0002
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        return self._send_request(constraints, wait)

class EasyRobot:
    def __init__(self, arm_group="left_arm", gripper_group="gripper", base_frame="world"):
        """
        初始化机器人
        :param arm_group: 机械臂规划组名称
        :param gripper_group: 夹爪规划组名称 (请查阅 SRDF 文件)
        """
        if not rclpy.ok():
            rclpy.init()
            
        self.node = rclpy.create_node("easy_robot_driver")
        
        # 1. 创建机械臂客户端
        self.arm = MoveItActionClient(self.node, arm_group, base_frame, external_spin=True)
        
        # 2. 创建夹爪客户端
        self.gripper = MoveItActionClient(self.node, gripper_group, base_frame, external_spin=True)
        
        # 启动后台线程
        self.spinner = threading.Thread(target=rclpy.spin, args=(self.node,))
        self.spinner.start()

    def go_to(self, x, y, z, qx=0, qy=0, qz=0, qw=1):
        """控制机械臂移动到坐标"""
        target = PoseStamped()
        target.header.frame_id = self.arm.base_frame
        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.position.z = float(z)
        target.pose.orientation.x = float(qx)
        target.pose.orientation.y = float(qy)
        target.pose.orientation.z = float(qz)
        target.pose.orientation.w = float(qw)
        return self.arm.move_to_pose(target)

    def set_gripper(self, value):
        """
        设置夹爪开合
        :param value: 0.0 ~ 1.0 (根据实际关节定义可能是弧度或米)
        注意：你需要知道夹爪的关节名称
        """
        # === 请根据你的 SRDF 修改这里的关节名称 ===
        # 常见的 OpenARM 夹爪关节名可能是: 'gripper_joint', 'finger_joint1' 等
        # 下面是一个假设的例子，如果运行报错说找不到关节，请修改这里
        joint_name = "openarm_left_finger_joint1" 
        
        # 有些夹爪有两个手指，需要同时驱动
        joints = {
            joint_name: value,
            # "openarm_finger_joint2": value # 如果有第二个手指
        }
        return self.gripper.move_to_joint(joints)

    def open_gripper(self):
        """快捷指令：张开"""
        # 根据实际情况修改数值，通常 0 是闭合，最大值是张开
        return self.set_gripper(0.044) 

    def close_gripper(self):
        """快捷指令：闭合"""
        return self.set_gripper(0.0)

    def close(self):
        rclpy.shutdown()
        self.spinner.join()