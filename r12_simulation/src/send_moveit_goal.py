#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_euler
from control_msgs.action import GripperCommand

# State definitions
STATE_MOVE_TO_POSE1 = 0
STATE_CLOSE_GRIPPER = 1
STATE_MOVE_TO_POSE2 = 2
STATE_OPEN_GRIPPER = 3
STATE_DONE = 4


class MoveitGoalSender(Node):
    def __init__(self):
        super().__init__('moveit_goal_sender')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._gripper_client = ActionClient(self, GripperCommand, '/hand_controller/gripper_cmd')

        self.get_logger().info('Waiting for MoveGroup action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Waiting for Gripper action server...')
        self._gripper_client.wait_for_server()

        self.state = STATE_MOVE_TO_POSE1
        self.target_poses = self._create_target_poses()

    def _create_target_poses(self):
        """Create a list of target poses to execute sequentially"""
        poses = []

        # First goal: Pick position
        pose1 = PoseStamped()
        pose1.header.frame_id = "base_link"
        pose1.pose.position.x = 0.27
        pose1.pose.position.y = 0.19
        pose1.pose.position.z = 0.34
        roll, pitch, yaw = 0.0, 3.14, 0.0
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose1.pose.orientation.x = qx
        pose1.pose.orientation.y = qy
        pose1.pose.orientation.z = qz
        pose1.pose.orientation.w = qw
        poses.append(pose1)

        # Second goal: Place position
        pose2 = PoseStamped()
        pose2.header.frame_id = "base_link"
        pose2.pose.position.x = 0.27
        pose2.pose.position.y = -0.19
        pose2.pose.position.z = 0.36
        roll, pitch, yaw = 0.0, 3.14, 0.0
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose2.pose.orientation.x = qx
        pose2.pose.orientation.y = qy
        pose2.pose.orientation.z = qz
        pose2.pose.orientation.w = qw
        poses.append(pose2)

        return poses

    def send_move_goal(self, target_pose: PoseStamped):
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()

        request.group_name = "r12_arm"
        request.goal_constraints.append(
            self._make_pose_constraint(target_pose, "wrist_tool_link")
        )
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        goal_msg.request = request

        self.get_logger().info(f'Sending move goal...')
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.move_goal_response_callback)

    def _make_pose_constraint(self, target_pose: PoseStamped, link_name: str):
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint

        constraints = Constraints()

        pos_con = PositionConstraint()
        pos_con.header = target_pose.header
        pos_con.link_name = link_name
        pos_con.target_point_offset.x = 0.0
        pos_con.target_point_offset.y = 0.0
        pos_con.target_point_offset.z = 0.0
        pos_con.constraint_region.primitive_poses.append(target_pose.pose)

        solid = SolidPrimitive()
        solid.type = SolidPrimitive.SPHERE
        solid.dimensions = [0.01]
        pos_con.constraint_region.primitives.append(solid)
        constraints.position_constraints.append(pos_con)

        ori_con = OrientationConstraint()
        ori_con.header = target_pose.header
        ori_con.link_name = link_name
        ori_con.orientation = target_pose.pose.orientation
        ori_con.absolute_x_axis_tolerance = 0.02
        ori_con.absolute_y_axis_tolerance = 0.02
        ori_con.absolute_z_axis_tolerance = 0.02
        ori_con.weight = 1.0
        constraints.orientation_constraints.append(ori_con)

        return constraints

    def move_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Move goal rejected')
            return

        self.get_logger().info('Move goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move_result_callback)

    def move_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Move completed with error code: {result.error_code.val}')

        if self.state == STATE_MOVE_TO_POSE1:
            self.get_logger().info('Moving to next state: closing gripper')
            self.state = STATE_CLOSE_GRIPPER
            self.close_gripper()
        elif self.state == STATE_MOVE_TO_POSE2:
            self.get_logger().info('Moving to next state: opening gripper')
            self.state = STATE_OPEN_GRIPPER
            self.open_gripper()

    def close_gripper(self):
        gripper_goal = GripperCommand.Goal()
        gripper_goal.command.position = -0.012  # adjust based on your gripper
        gripper_goal.command.max_effort = 10.0

        self.get_logger().info('Sending gripper close command...')
        future = self._gripper_client.send_goal_async(gripper_goal)
        future.add_done_callback(self.gripper_close_response_callback)

    def gripper_close_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper close goal rejected')
            return

        self.get_logger().info('Gripper close goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_close_result_callback)

    def gripper_close_result_callback(self, future):
        self.get_logger().info('Gripper closed successfully!')
        self.get_logger().info('Moving to second position...')
        self.state = STATE_MOVE_TO_POSE2
        self.send_move_goal(self.target_poses[1])

    def open_gripper(self):
        gripper_goal = GripperCommand.Goal()
        gripper_goal.command.position = 0.0  # fully open
        gripper_goal.command.max_effort = 10.0

        self.get_logger().info('Sending gripper open command...')
        future = self._gripper_client.send_goal_async(gripper_goal)
        future.add_done_callback(self.gripper_open_response_callback)

    def gripper_open_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper open goal rejected')
            return

        self.get_logger().info('Gripper open goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_open_result_callback)

    def gripper_open_result_callback(self, future):
        self.get_logger().info('Gripper opened successfully!')
        self.get_logger().info('All tasks completed.')
        rclpy.shutdown()

    def start(self):
        self.get_logger().info('Starting task: moving to first pose')
        self.send_move_goal(self.target_poses[0])


def main(args=None):
    rclpy.init(args=args)
    node = MoveitGoalSender()
    node.start()
    rclpy.spin(node)


if __name__ == '__main__':
    main()