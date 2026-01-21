import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup # 关键导入
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MotionPlanRequest, PositionConstraint, OrientationConstraint, Constraints
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
import threading
import time  # 必须导入 time

class MoveItActionClient:
    def __init__(self, node: Node, group_name="left_arm", base_frame="base_link", external_spin=False):
        self.node = node
        self.group_name = group_name
        self.base_frame = base_frame
        self.external_spin = external_spin
        
        # 关键修改：使用 ReentrantCallbackGroup
        # 这允许 Action 的回调在后台线程中并行处理，防止死锁
        self._cb_group = ReentrantCallbackGroup()
        
        self._action_client = ActionClient(
            node, 
            MoveGroup, 
            'move_action',
            callback_group=self._cb_group
        )
        
        self.node.get_logger().info(f"等待 MoveGroup Action 服务器 ({group_name})...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveGroup Action 服务器未上线！请确保运行了 demo.launch.py")
        else:
            self.node.get_logger().info("MoveGroup Action 服务器已连接")

    def _wait_for_future(self, future, timeout=10.0):
        """
        更稳健的等待逻辑，防止 returning None
        """
        if self.external_spin:
            # 在多线程模式下，手动循环等待，这比 future.result() 更可靠
            start_time = time.time()
            while not future.done():
                time.sleep(0.01) # 休息 10ms，避免占用 CPU
                if time.time() - start_time > timeout:
                    self.node.get_logger().error("等待 Action 响应超时！")
                    return None
            return future.result()
        else:
            # 单线程模式
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            return future.result()

    def move_to_pose(self, target_pose: PoseStamped, wait=True):
        goal_msg = MoveGroup.Goal()
        
        # 1. 设置规划请求
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 1.0
        goal_msg.request.max_acceleration_scaling_factor = 1.0

        # 2. 构建约束 (使用你之前测试成功的 link)
        link_name = "openarm_left_link7" 

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
        goal_msg.request.goal_constraints.append(constraints)

        # 3. 发送目标
        self.node.get_logger().info("正在发送规划请求...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # 等待服务器响应
        goal_handle = self._wait_for_future(send_goal_future)

        # 关键修改：增加空值检查
        if goal_handle is None:
            self.node.get_logger().error("未能获取 Goal Handle (可能是超时或通信错误)")
            return False

        if not goal_handle.accepted:
            self.node.get_logger().error('规划请求被拒绝 (目标点可能不可达)')
            return False

        self.node.get_logger().info('规划中并执行...')
        
        if wait:
            get_result_future = goal_handle.get_result_async()
            result_wrapper = self._wait_for_future(get_result_future, timeout=30.0)
            
            # 再次检查空值
            if result_wrapper is None:
                self.node.get_logger().error("等待执行结果超时！")
                return False

            result = result_wrapper.result
            
            if result.error_code.val == 1:
                self.node.get_logger().info('执行成功!')
                return True
            else:
                self.node.get_logger().error(f'执行失败，错误码: {result.error_code.val}')
                return False
        return True

class EasyRobot:
    def __init__(self, group_name="left_arm", base_frame="world"):
        if not rclpy.ok():
            rclpy.init()
            
        self.node = rclpy.create_node("easy_robot_driver")
        
        self.client = MoveItActionClient(
            self.node, 
            group_name, 
            base_frame, 
            external_spin=True 
        )
        
        self.spinner = threading.Thread(target=rclpy.spin, args=(self.node,))
        self.spinner.start()

    def go_to(self, x, y, z, qx=0, qy=0, qz=0, qw=1):
        target = PoseStamped()
        target.header.frame_id = self.client.base_frame
        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.position.z = float(z)
        target.pose.orientation.x = float(qx)
        target.pose.orientation.y = float(qy)
        target.pose.orientation.z = float(qz)
        target.pose.orientation.w = float(qw)
        
        return self.client.move_to_pose(target)

    def close(self):
        rclpy.shutdown()
        self.spinner.join()