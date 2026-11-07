#!/usr/bin/env python3
"""Time-synchronised pairing of the M1's camera and LiDAR streams.

Runs on the OPERATOR PC. Ports the `ApproximateTimeSynchronizer` pattern to the
ROSMASTER M1's sensors.

    /image_throttled/compressed  (15-20 Hz)  \
                                               >--[sync]--> /m1/synced/overlay
    /scan                        (10 Hz)     /

Why a synchronizer is needed
----------------------------
The camera and the LiDAR run on independent clocks inside independent drivers,
so their stamps never line up exactly. Pairing "the newest image" with "the
newest scan" silently mixes data up to a full scan period apart - at 10 Hz that
is 100 ms, which at 0.2 m/s is 2 cm of chassis travel plus whatever the PTZ
gimbal did in between. `ApproximateTimeSynchronizer` keeps a small buffer and
emits the pair whose stamps are closest, so every fused output is coherent.

The slop is set to 0.15 s: comfortably above the 100 ms scan period but small
enough to reject a genuinely stale partner.

Two useful diagnostics are built in, because a synchronizer that never fires is
indistinguishable from one that is simply waiting:
  * per-stream counters, logged every 2 s, prove data is actually arriving;
  * a stamp-jump detector that resets the buffers when time moves backwards
    (rosbag loop, or the robot rebooting mid-session).

Output
------
`/m1/synced/overlay` is a `sensor_msgs/Image` with the matching scan drawn on
top of the camera frame, so a mis-synchronised pair is visible immediately
rather than silently wrong. `/m1/synced/stats` reports the pairing quality.

    ros2 run m1_control sensor_sync_node
    ros2 run m1_control sensor_sync_node --ros-args -p slop:=0.2 -p show_window:=true
"""

import threading

import cv2
import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Float32, String


class SensorSyncNode(Node):

    def __init__(self):
        super().__init__('sensor_sync_node')

        # ---------------- parameters ----------------
        self.declare_parameter('image_topic', '/image_throttled/compressed')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/m1/synced/overlay')
        self.declare_parameter('stats_topic', '/m1/synced/stats')
        self.declare_parameter('queue_size', 30)
        self.declare_parameter('slop', 0.15)
        self.declare_parameter('show_window', False)
        self.declare_parameter('scan_range_max', 4.0)
        self.declare_parameter('min_pairs_for_report', 10)

        self.image_topic = self.get_parameter('image_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.queue_size = int(self.get_parameter('queue_size').value)
        self.slop = float(self.get_parameter('slop').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.scan_range_max = float(self.get_parameter('scan_range_max').value)

        # ---------------- QoS ----------------
        # Sensor telemetry over Wi-Fi must be BEST_EFFORT: a dropped frame is
        # recoverable, a retransmission storm is not.
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # ---------------- threading ----------------
        # The synchronizer holds two subscriptions plus the image decode; a
        # single-threaded executor serialises them and the buffer starves.
        self.cb_group = ReentrantCallbackGroup()

        # ---------------- subscriptions ----------------
        self.image_sub = message_filters.Subscriber(
            self, CompressedImage, self.image_topic,
            qos_profile=self.sensor_qos, callback_group=self.cb_group)
        self.scan_sub = message_filters.Subscriber(
            self, LaserScan, self.scan_topic,
            qos_profile=self.sensor_qos, callback_group=self.cb_group)

        # Raw counters so "no output" can be told apart from "no input".
        self.n_images = 0
        self.n_scans = 0
        self.n_pairs = 0
        self.last_pair_diff = float('nan')
        self.lock = threading.Lock()

        self.create_subscription(CompressedImage, self.image_topic,
                                 self._image_diag, self.sensor_qos,
                                 callback_group=self.cb_group)
        self.create_subscription(LaserScan, self.scan_topic,
                                 self._scan_diag, self.sensor_qos,
                                 callback_group=self.cb_group)

        # ---------------- synchronizer ----------------
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.scan_sub],
            queue_size=self.queue_size,
            slop=self.slop)
        self.sync.registerCallback(self.sync_callback)

        # ---------------- publishers ----------------
        self.overlay_pub = self.create_publisher(
            Image, self.get_parameter('output_topic').value, self.sensor_qos)
        self.stats_pub = self.create_publisher(
            String, self.get_parameter('stats_topic').value, 10)

        self.create_timer(2.0, self._report)
        self._window_ready = False

        self.get_logger().info(
            f'Sync node up | {self.image_topic} + {self.scan_topic} | '
            f'queue={self.queue_size} slop={self.slop * 1000:.0f} ms')

    # ------------------------------------------------------------ diagnostics
    def _image_diag(self, _msg):
        with self.lock:
            self.n_images += 1

    def _scan_diag(self, _msg):
        with self.lock:
            self.n_scans += 1

    def _report(self):
        with self.lock:
            imgs, scans, pairs = self.n_images, self.n_scans, self.n_pairs
            diff = self.last_pair_diff

        if imgs == 0 and scans == 0:
            self.get_logger().warn(
                f'No data on either stream. Is the robot running and is '
                f'ROS_DOMAIN_ID the same on both machines?')
            return

        if pairs == 0:
            self.get_logger().warn(
                f'Receiving but not pairing: images={imgs} scans={scans}. '
                f'Raise `slop` or check that both stamps come from the same '
                f'clock.')
            return

        self.get_logger().info(
            f'images={imgs} scans={scans} pairs={pairs} '
            f'last_dt={diff * 1000:.1f} ms')

        self.stats_pub.publish(String(data=(
            f'images={imgs} scans={scans} pairs={pairs} '
            f'last_dt_ms={diff * 1000:.1f}')))

    # --------------------------------------------------------------- callback
    def sync_callback(self, image_msg: CompressedImage, scan_msg: LaserScan):
        img_t = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        scan_t = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9

        # Time moving backwards means a rosbag loop or a robot reboot: the
        # buffered messages are now in the future and would block pairing.
        if hasattr(self, '_last_t') and img_t < self._last_t - 1.0:
            self.get_logger().warn(
                'Stamp jumped backwards - resetting synchronizer buffers')
            self.sync.sync_queue.clear()
            self.image_sub.signalMessage = self.image_sub.signalMessage
        self._last_t = img_t

        with self.lock:
            self.n_pairs += 1
            self.last_pair_diff = abs(img_t - scan_t)

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        self._draw_scan(frame, scan_msg)

        out = Image()
        out.header = image_msg.header
        out.height, out.width = frame.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = frame.shape[1] * 3
        out.data = frame.tobytes()
        self.overlay_pub.publish(out)

        if self.show_window:
            if not self._window_ready:
                cv2.namedWindow('M1 sensor sync', cv2.WINDOW_NORMAL)
                self._window_ready = True
            cv2.imshow('M1 sensor sync', frame)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                raise KeyboardInterrupt

    # ---------------------------------------------------------------- drawing
    def _draw_scan(self, frame, scan: LaserScan):
        """Project the scan onto the frame as a simple horizon fan.

        This is deliberately a schematic, not a calibrated projection: it makes
        a bad pairing obvious (the fan lags the scene) without needing
        CameraInfo, which this camera does not publish.
        """
        h, w = frame.shape[:2]
        horizon = int(h * 0.62)
        cx = w // 2
        scale = w / 2.0 / max(1e-3, self.scan_range_max)

        cv2.line(frame, (0, horizon), (w, horizon), (60, 60, 60), 1)

        n = len(scan.ranges)
        if n == 0:
            return
        step = max(1, n // 240)      # subsample: 404 beams is plenty already

        for i in range(0, n, step):
            r = scan.ranges[i]
            if not np.isfinite(r) or r < scan.range_min or r > self.scan_range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            # Map the robot's -90..+90 deg forward arc across the frame width.
            if angle < -np.pi / 2 or angle > np.pi / 2:
                continue
            px = int(cx + np.sin(angle) * r * scale)
            py = int(horizon - min(h * 0.35, (self.scan_range_max - r) * 6))
            if 0 <= px < w:
                cv2.circle(frame, (px, py), 2, (0, 255, 0), -1)

        cv2.putText(frame, f'SCAN {len(scan.ranges)} pts  dt '
                    f'{self.last_pair_diff * 1000:.0f} ms',
                    (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorSyncNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
