#!/usr/bin/env python3
"""UR3 Robotiq 2F-140 TCP setpoint bridge. No automatic movement at startup.

ROS input:  /ur3_robotiq/command (std_msgs/Float64MultiArray): [opening_mm, speed_norm, force_norm]
ROS output: /ur3_robotiq/status (std_msgs/String): transport state only, NOT physical feedback
For the existing PolyScope `socket_read_ascii_float(6, "grip_socket", ...)`.
"""
import math
import queue
import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_msgs.msg import Float64


def encode_command(position_mm, speed, force):
    values = (float(position_mm), float(speed), float(force))
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Command values must be finite')
    if not 0 <= values[0] <= 140:
        raise ValueError('opening_mm must be 0..140')
    if not 0 <= values[1] <= 100 or not 0 <= values[2] <= 100:
        raise ValueError('speed_norm/force_norm must be 0..100')
    # URScript socket_read_ascii_float protocol: (v1,v2,v3) + newline
    return ('(%.3f,%.3f,%.3f)\n' % values).encode('ascii')


class UR3RobotiqController(Node):
    def __init__(self):
        super().__init__('ur3_robotiq_controller')
        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('port', 5001)
        self.host = self.get_parameter('bind_host').value
        self.port = int(self.get_parameter('port').value)
        self._stop = threading.Event()
        self._pending = queue.Queue(maxsize=1)
        self._status = 'STARTING'
        self._lock = threading.Lock()
        self.create_subscription(Float64MultiArray, 'ur3_robotiq/command', self._command, 10)
        self.status_pub = self.create_publisher(String, 'ur3_robotiq/status', 10)
        self.visual_pub = self.create_publisher(Float64, 'robotiq_visual/position', 10)
        self.create_timer(0.5, self._publish_status)
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _set_status(self, state):
        with self._lock:
            self._status = state

    def _publish_status(self):
        with self._lock:
            state = self._status
        msg = String()
        msg.data = state
        self.status_pub.publish(msg)

    def _command(self, message):
        if len(message.data) != 3:
            self.get_logger().error('Expected exactly [opening_mm, speed_norm, force_norm]')
            return
        try:
            payload = encode_command(*message.data)
        except (ValueError, TypeError) as exc:
            self.get_logger().error('Rejected gripper command: %s' % exc)
            return
        # Latest setpoint wins; never replay an old command after reconnect.
        try:
            self._pending.get_nowait()
        except queue.Empty:
            pass
        self._pending.put_nowait((payload, message.data[0]))

    def _serve(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.host, self.port))
                server.listen(1)
                server.settimeout(0.25)
                self._set_status('LISTENING')
                self.get_logger().info('Robotiq server listening on %s:%s' % (self.host, self.port))
                while not self._stop.is_set():
                    try:
                        client, peer = server.accept()
                    except socket.timeout:
                        continue
                    with client:
                        client.settimeout(0.25)
                        self._set_status('CONNECTED')
                        self.get_logger().info('PolyScope gripper socket connected: %s' % (peer,))
                        # Discard commands queued before client was connected.
                        while not self._pending.empty():
                            try:
                                self._pending.get_nowait()
                            except queue.Empty:
                                break
                        while not self._stop.is_set():
                            try:
                                payload, position_mm = self._pending.get(timeout=0.1)
                            except queue.Empty:
                                # Detect closure, even with no commands.
                                try:
                                    data = client.recv(1, socket.MSG_PEEK | socket.MSG_DONTWAIT)
                                    if data == b'':
                                        break
                                except (BlockingIOError, socket.timeout):
                                    pass
                                except OSError:
                                    break
                                continue
                            try:
                                client.sendall(payload)
                            except OSError:
                                break
                            # Visual command estimate only, not measured gripper state.
                            vis = Float64()
                            vis.data = max(0., min(1., float(position_mm) / 140.))
                            self.visual_pub.publish(vis)
                        self._set_status('LISTENING')
        except OSError as exc:
            self._set_status('ERROR: %s' % exc)
            self.get_logger().error('Robotiq server: %s' % exc)

    def destroy_node(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UR3RobotiqController()
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
