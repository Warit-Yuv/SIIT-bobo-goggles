#!/usr/bin/env python3
"""Compressed-image rate limiter (drop-in replacement for `topic_tools throttle`).

`topic_tools` is not installed in this ROS 2 Humble image, so the on-robot
bandwidth cap is implemented here instead of relying on the missing package.

    /image_raw/compressed  -->  [throttle]  -->  /image_throttled/compressed

Frames arriving faster than `target_rate_hz` are dropped, never queued: for a
live video feed the newest frame is always the interesting one. The publisher
is BEST_EFFORT so a slow Wi-Fi link degrades by dropping frames instead of
triggering DDS retransmission storms.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class ImageThrottleNode(Node):

    def __init__(self):
        super().__init__('image_throttle_node')

        self.declare_parameter('input_topic', '/image_raw/compressed')
        self.declare_parameter('output_topic', '/image_throttled/compressed')
        self.declare_parameter('target_rate_hz', 20.0)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.min_period = 1.0 / max(1e-3, float(self.get_parameter('target_rate_hz').value))

        # Input: accept whatever the camera offers, keep only the newest frame.
        self.create_subscription(
            CompressedImage, in_topic, self.cb,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        # Output: BEST_EFFORT over Wi-Fi (a dropped frame is better than a stall).
        self.pub = self.create_publisher(
            CompressedImage, out_topic,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        self.last_emit = 0.0
        self.credit = 0.0
        self.prev_stamp = None
        self.dropped = 0
        self.emitted = 0
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f'Throttling {in_topic} -> {out_topic} '
            f'@ {self.get_parameter("target_rate_hz").value:.1f} Hz (BEST_EFFORT)'
        )

    def cb(self, msg: CompressedImage):
        now = self.get_clock().now().nanoseconds / 1e9

        # Time-credit accumulator.
        #
        # A plain "minimum period" limiter quantises to input/N: from a 30 Hz
        # source with a 20 Hz target it emits 15 Hz, because after emitting it
        # must skip the very next frame (33 ms < 50 ms) and then waits again.
        # Accumulating the elapsed time as credit lets it emit two frames per
        # 100 ms (66 + 33 ms) and then skip one, giving a true 20 Hz average.
        #
        # The credit cap must be at least 2 periods. Capping at 1 period
        # throws away the leftover credit on every emit, which collapses the
        # output straight back to input/2 (measured: 15.00 Hz from a 30 Hz
        # source). 2 periods keeps the fractional remainder and reaches the
        # target (measured: 19.90 Hz).
        if self.prev_stamp is None:
            dt = 0.0
        else:
            dt = max(0.0, now - self.prev_stamp)
        self.prev_stamp = now

        # Guard against a long stall (node paused, Wi-Fi gap) turning into a
        # burst: never credit more than 200 ms in one step.
        self.credit = min(self.credit + min(dt, 0.2), self.min_period * 2.0)

        if self.credit < self.min_period:
            self.dropped += 1
            return

        self.credit -= self.min_period
        self.last_emit = now
        self.emitted += 1
        self.pub.publish(msg)

    def report(self):
        if self.emitted or self.dropped:
            self.get_logger().debug(
                f'sent={self.emitted} dropped={self.dropped} '
                f'({100.0 * self.dropped / max(1, self.emitted + self.dropped):.1f}% dropped)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImageThrottleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
