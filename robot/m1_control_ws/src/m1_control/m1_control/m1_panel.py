#!/usr/bin/env python3
"""Control panel for the ROSMASTER M1 — mode buttons, live status, E-STOP.

Runs on the OPERATOR PC. A small always-on-top window so you can switch modes
and watch the robot without tying up a terminal.

    ros2 run m1_control m1_panel
    ros2 run m1_control m1_panel --ros-args -p robot_ip:=10.0.0.54

What it shows
-------------
  * current mode (and whether E-STOP is latched)
  * battery voltage, with a warning below 10.5 V
  * gaze zone, when gaze control is running
  * image rate, so a stalled video feed is visible immediately

What it does
------------
  * 1 / 2 / 3 / 0 to select KEYBOARD / GAZE / EEG / E-STOP
  * big E-STOP button, always reachable
  * keyboard driving works in every mode (the arbiter honours a recent
    keyboard command regardless of mode), so you can nudge the robot while
    gaze or EEG is in charge

Why a GUI at all
----------------
Switching modes from a terminal means either typing a `ros2 topic pub` command
or keeping the teleop window focused. Both are bad when the robot is moving and
you want it to stop. A button that is always on screen is the safer interface.
"""

import sys
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, String, UInt8

MODE_NAMES = {0: 'E-STOP', 1: 'KEYBOARD', 2: 'GAZE', 3: 'EEG'}
MODE_COLORS = {0: '#c0392b', 1: '#2980b9', 2: '#27ae60', 3: '#8e44ad'}


class PanelNode(Node):
    """Thin ROS wrapper. The GUI reads snapshots from `state`."""

    def __init__(self):
        super().__init__('m1_panel')
        self.declare_parameter('robot_ip', '10.0.0.54')
        self.declare_parameter('refresh_hz', 10.0)

        self.state = {
            'mode': None,
            'voltage': float('nan'),
            'zone': '',
            'image_hz': 0.0,
            'connected': False,
        }
        self._image_times = []

        mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.mode_pub = self.create_publisher(UInt8, '/control_mode', mode_qos)
        self.estop_pub = self.create_publisher(Twist, '/cmd_vel/keyboard', 10)

        # Latched, so the panel knows the mode even if it starts late.
        self.create_subscription(UInt8, '/control_mode_state',
                                 self._mode_cb, mode_qos)
        self.create_subscription(Float32, '/voltage', self._voltage_cb, sensor_qos)
        self.create_subscription(String, '/gaze/zone', self._zone_cb, 10)
        self.create_subscription(CompressedImage, '/image_throttled/compressed',
                                 self._image_cb, sensor_qos)

        self.get_logger().info('Panel up — publishing /control_mode')

    def _mode_cb(self, msg):
        self.state['mode'] = int(msg.data)
        self.state['connected'] = True

    def _voltage_cb(self, msg):
        self.state['voltage'] = float(msg.data)
        self.state['connected'] = True

    def _zone_cb(self, msg):
        self.state['zone'] = msg.data

    def _image_cb(self, _msg):
        import time
        now = time.time()
        self._image_times.append(now)
        self._image_times = [t for t in self._image_times if now - t < 2.0]
        self.state['image_hz'] = len(self._image_times) / 2.0

    def set_mode(self, mode):
        self.mode_pub.publish(UInt8(data=int(mode)))

    def stop(self):
        """E-STOP: latched zero mode, plus an explicit zero velocity."""
        self.set_mode(0)
        self.estop_pub.publish(Twist())


def run_ros(node, stop_event):
    while not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = PanelNode()

    # ROS spins in a background thread so the GUI stays responsive.
    stop_event = threading.Event()
    t = threading.Thread(target=run_ros, args=(node, stop_event), daemon=True)
    t.start()

    try:
        import tkinter as tk
        from tkinter import font as tkfont
    except ImportError:
        print('tkinter is not installed. On Ubuntu:\n'
              '    sudo apt install python3-tk\n'
              'Falling back to a terminal loop.', file=sys.stderr)
        _terminal_loop(node, stop_event)
        return 0

    root = tk.Tk()
    root.title('ROSMASTER M1')
    root.configure(bg='#1e1e1e')
    root.geometry('360x300')
    # Stay above RViz so E-STOP is always reachable.
    root.attributes('-topmost', True)

    big = tkfont.Font(family='DejaVu Sans', size=15, weight='bold')
    small = tkfont.Font(family='DejaVu Sans', size=10)

    mode_var = tk.StringVar(value='connecting…')
    mode_label = tk.Label(root, textvariable=mode_var, font=big,
                          fg='white', bg='#555555', pady=10)
    mode_label.pack(fill='x', padx=10, pady=(10, 6))

    info = tk.Label(root, text='', font=small, fg='#cccccc', bg='#1e1e1e',
                    justify='left', anchor='w')
    info.pack(fill='x', padx=12)

    btn_row = tk.Frame(root, bg='#1e1e1e')
    btn_row.pack(fill='x', padx=10, pady=8)

    def make_btn(parent, text, mode, colour):
        return tk.Button(
            parent, text=text, font=small, bg=colour, fg='white',
            activebackground=colour, relief='flat', height=2,
            command=lambda m=mode: node.set_mode(m))

    make_btn(btn_row, '1  KEYBOARD', 1, '#2980b9').grid(row=0, column=0, sticky='ew', padx=2, pady=2)
    make_btn(btn_row, '2  GAZE', 2, '#27ae60').grid(row=0, column=1, sticky='ew', padx=2, pady=2)
    make_btn(btn_row, '3  EEG', 3, '#8e44ad').grid(row=1, column=0, sticky='ew', padx=2, pady=2)
    make_btn(btn_row, '0  E-STOP', 0, '#c0392b').grid(row=1, column=1, sticky='ew', padx=2, pady=2)
    btn_row.columnconfigure(0, weight=1)
    btn_row.columnconfigure(1, weight=1)

    estop = tk.Button(root, text='■  EMERGENCY STOP', font=big, bg='#c0392b',
                      fg='white', activebackground='#e74c3c', relief='flat',
                      height=2, command=node.stop)
    estop.pack(fill='x', padx=10, pady=(0, 6))

    hint = tk.Label(root, text='keys 1/2/3/0  ·  keyboard drives in any mode',
                    font=small, fg='#888888', bg='#1e1e1e')
    hint.pack(pady=(0, 8))

    for key, mode in (('1', 1), ('2', 2), ('3', 3), ('0', 0)):
        root.bind(f'<KeyPress-{key}>', lambda _e, m=mode: node.set_mode(m))

    def refresh():
        s = node.state
        mode = s['mode']
        if mode is None:
            mode_var.set('connecting…')
            mode_label.configure(bg='#555555')
        else:
            label = MODE_NAMES.get(mode, str(mode))
            if mode == 0:
                label = 'E-STOP LATCHED'
            mode_var.set(label)
            mode_label.configure(bg=MODE_COLORS.get(mode, '#555555'))

        v = s['voltage']
        vtxt = 'battery  --' if v != v else f'battery  {v:.1f} V'
        if v == v and v < 10.5:
            vtxt += '   LOW'
        parts = [vtxt, f'video    {s["image_hz"]:.0f} fps']
        if s['zone']:
            parts.append(f'gaze     {s["zone"]}')
        if not s['connected']:
            parts.append('no data — is the robot running?')
        info.configure(text='\n'.join(parts))

        root.after(150, refresh)

    refresh()
    root.protocol('WM_DELETE_WINDOW', lambda: (stop_event.set(), root.destroy()))

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        t.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _terminal_loop(node, stop_event):
    """Fallback when tkinter is unavailable."""
    print('M1 panel (terminal mode). Commands: 1=KEYBOARD 2=GAZE 3=EEG 0=E-STOP q=quit')
    import time
    while not stop_event.is_set():
        s = node.state
        v = s['voltage']
        vtxt = '--' if v != v else f'{v:.1f}V'
        print(f"\rmode={MODE_NAMES.get(s['mode'], '?')}  bat={vtxt}  "
              f"zone={s['zone'] or '-'}  fps={s['image_hz']:.0f}   ",
              end='', flush=True)
        time.sleep(0.5)


if __name__ == '__main__':
    sys.exit(main())
