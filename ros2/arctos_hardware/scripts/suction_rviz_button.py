#!/usr/bin/env python3
"""
Interactive-marker toggle button for the Arctos suction gripper.

Publishes a clickable cube + label under the `suction_button` interactive-marker
namespace. Clicking the marker in RViz (with the Interact tool active) toggles
`/suction/activate`; the marker turns green when OFF and red when ON, and
re-syncs from `/suction/status` so it reflects the real driver state.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import SetBool
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)
from interactive_markers import InteractiveMarkerServer


BUTTON_FRAME = 'base_link'
BUTTON_POSITION = (0.40, -0.40, 0.40)
BUTTON_SIZE = 0.12

OFF_RGB = (0.20, 0.70, 0.25)
ON_RGB = (0.85, 0.20, 0.20)


class SuctionRvizButton(Node):
    def __init__(self) -> None:
        super().__init__('suction_rviz_button')

        self._server = InteractiveMarkerServer(self, 'suction_button')
        self._client = self.create_client(SetBool, '/suction/activate')
        self.create_subscription(
            UInt8MultiArray, '/suction/status', self._status_cb, 10
        )

        self._state_on = False
        self._pending = False

        self._publish_marker()
        self.get_logger().info(
            'Suction RViz button ready — add an InteractiveMarkers display '
            "on topic 'suction_button' and click the cube in the scene."
        )

    def _status_cb(self, msg: UInt8MultiArray) -> None:
        if not msg.data:
            return
        reported = bool(msg.data[0])
        if reported != self._state_on:
            self._state_on = reported
            self._publish_marker()

    def _publish_marker(self) -> None:
        im = InteractiveMarker()
        im.header.frame_id = BUTTON_FRAME
        im.name = 'suction_toggle'
        im.description = ''
        im.pose.position.x, im.pose.position.y, im.pose.position.z = BUTTON_POSITION
        im.pose.orientation.w = 1.0
        im.scale = BUTTON_SIZE * 1.5

        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = BUTTON_SIZE
        cube.scale.y = BUTTON_SIZE
        cube.scale.z = BUTTON_SIZE * 0.5
        r, g, b = ON_RGB if self._state_on else OFF_RGB
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = r, g, b, 1.0

        label = Marker()
        label.type = Marker.TEXT_VIEW_FACING
        label.text = 'SUCTION ON' if self._state_on else 'SUCTION OFF'
        label.scale.z = BUTTON_SIZE * 0.4
        label.pose.position.z = BUTTON_SIZE * 0.7
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0

        control = InteractiveMarkerControl()
        control.name = 'button'
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append(cube)
        control.markers.append(label)

        im.controls.append(control)
        self._server.insert(im, feedback_callback=self._on_feedback)
        self._server.applyChanges()

    def _on_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return
        if self._pending:
            return
        if not self._client.service_is_ready():
            self.get_logger().warn('/suction/activate not available yet')
            return

        target = not self._state_on
        self._pending = True
        request = SetBool.Request()
        request.data = target
        future = self._client.call_async(request)
        future.add_done_callback(lambda fut: self._on_service_done(fut, target))

    def _on_service_done(self, future, target: bool) -> None:
        self._pending = False
        result = future.result()
        if result is None:
            self.get_logger().warn('Suction activate call returned no result')
            return
        if not result.success:
            self.get_logger().warn(f'Suction activate failed: {result.message}')
            return
        if target != self._state_on:
            self._state_on = target
            self._publish_marker()


def main() -> None:
    rclpy.init()
    node = SuctionRvizButton()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
