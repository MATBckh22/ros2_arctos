#!/usr/bin/env python3
"""
GripperCommand action server for the Arctos suction gripper.

Bridges MoveIt's standard GripperCommand interface to the suction driver's
ROS services, and publishes a visual marker at the suction tip in RViz.

MoveIt mapping:
  position >= 0.5 → suction ON  (grip)
  position <  0.5 → suction OFF (release)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import GripperCommand
from std_srvs.srv import SetBool
from std_msgs.msg import UInt8MultiArray, ColorRGBA
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration


class SuctionGripperAction(Node):
    def __init__(self):
        super().__init__('suction_gripper_action')

        self._cb_group = ReentrantCallbackGroup()
        self._suction_active = False
        self._fault = False

        self._activate_client = self.create_client(
            SetBool, '/suction/activate', callback_group=self._cb_group
        )

        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/suction_gripper_controller/gripper_cmd',
            self._execute_callback,
            callback_group=self._cb_group,
        )

        self.create_subscription(
            UInt8MultiArray, '/suction/status', self._status_cb, 10
        )

        self._marker_pub = self.create_publisher(Marker, '/suction/marker', 10)
        self.create_timer(0.5, self._publish_marker)

        self.get_logger().info('Suction GripperCommand action server ready')

    def _status_cb(self, msg: UInt8MultiArray):
        if len(msg.data) >= 3:
            self._suction_active = msg.data[0] > 0
            self._fault = msg.data[2] != 0

    def _execute_callback(self, goal_handle):
        target_position = goal_handle.request.command.position
        activate = target_position >= 0.5

        self.get_logger().info(
            f'Gripper command: {"GRIP" if activate else "RELEASE"} '
            f'(position={target_position:.2f})'
        )

        if not self._activate_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Suction activate service not available')
            goal_handle.abort()
            return GripperCommand.Result(
                position=0.0, stalled=False, reached_goal=False
            )

        req = SetBool.Request(data=activate)
        future = self._activate_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None or not future.result().success:
            self.get_logger().error('Suction command failed')
            goal_handle.abort()
            return GripperCommand.Result(
                position=0.0, stalled=True, reached_goal=False
            )

        self._suction_active = activate
        result_position = 1.0 if activate else 0.0

        feedback = GripperCommand.Feedback(
            position=result_position, stalled=False, reached_goal=True
        )
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        self.get_logger().info(f'Gripper result: position={result_position}')
        return GripperCommand.Result(
            position=result_position, stalled=False, reached_goal=True
        )

    def _publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'C_Core_Suction'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'suction_state'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.z = 0.01
        marker.scale.x = 0.025
        marker.scale.y = 0.025
        marker.scale.z = 0.025
        marker.lifetime = Duration(sec=1)

        if self._fault:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
        elif self._suction_active:
            marker.color = ColorRGBA(r=0.1, g=0.9, b=0.3, a=0.9)
        else:
            marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5)

        self._marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = SuctionGripperAction()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
