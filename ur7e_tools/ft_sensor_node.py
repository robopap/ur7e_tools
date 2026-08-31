#!/usr/bin/env python3

import struct
import threading
import time
from collections import deque

import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger


START_STREAM_COMMAND = bytes.fromhex(
    "09 10 01 9A 00 01 02 02 00 CD CA"
)

FRAME_HEADER = b"\x20\x4e"
FRAME_SIZE = 16

FORCE_SCALE = 100.0
TORQUE_SCALE = 1000.0


def modbus_crc16(data: bytes) -> int:
    """Standard Modbus CRC-16."""
    crc = 0xFFFF

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc & 0xFFFF


class RobotiqFTStreamNode(Node):
    def __init__(self):
        super().__init__("robotiq_ft_sensor")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 19200)
        self.declare_parameter("frame_id", "external_ft_sensor")
        self.declare_parameter("tare_samples", 100)

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.tare_samples = max(
            1,
            int(self.get_parameter("tare_samples").value),
        )

        self.publisher = self.create_publisher(
            WrenchStamped,
            "/external_ft",
            10,
        )

        self.zero_service = self.create_service(
            Trigger,
            "/external_ft/zero",
            self.zero_callback,
        )

        self.offset_lock = threading.Lock()
        self.samples_lock = threading.Lock()

        self.offset = [0.0] * 6
        self.recent_samples = deque(maxlen=self.tare_samples)

        self.stop_event = threading.Event()
        self.reader_thread = None

        self.serial_port = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
        )

        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

        self._start_stream()

        self.reader_thread = threading.Thread(
            target=self._reader_loop,
            name="robotiq_ft_stream_reader",
            daemon=True,
        )
        self.reader_thread.start()

        self.get_logger().info(
            f"Robotiq F/T data stream started on {self.port} "
            f"({self.baudrate} baud). Publishing /external_ft."
        )

    def _start_stream(self):
        self.serial_port.write(START_STREAM_COMMAND)
        self.serial_port.flush()

        # Give the sensor a short moment to switch from Modbus RTU
        # command handling into continuous data-stream mode.
        time.sleep(0.05)

    def _stop_stream(self):
        if not hasattr(self, "serial_port"):
            return

        if not self.serial_port.is_open:
            return

        # Robotiq specifies interrupting the stream with 0xFF
        # characters for about 0.5 s.
        for _ in range(50):
            try:
                self.serial_port.write(b"\xff")
                self.serial_port.flush()
            except serial.SerialException:
                break

            time.sleep(0.01)

    def _reader_loop(self):
        buffer = bytearray()

        while not self.stop_event.is_set():
            try:
                waiting = self.serial_port.in_waiting
                chunk = self.serial_port.read(
                    waiting if waiting > 0 else 1
                )
            except serial.SerialException as exc:
                self.get_logger().error(
                    f"Serial read error: {exc}"
                )
                break

            if not chunk:
                continue

            buffer.extend(chunk)

            while True:
                header_index = buffer.find(FRAME_HEADER)

                if header_index < 0:
                    # Keep at most one trailing byte in case it is
                    # the first byte of the next 0x20 0x4E header.
                    if len(buffer) > 1:
                        del buffer[:-1]
                    break

                if header_index > 0:
                    del buffer[:header_index]

                if len(buffer) < FRAME_SIZE:
                    break

                frame = bytes(buffer[:FRAME_SIZE])

                expected_crc = (
                    frame[14]
                    | (frame[15] << 8)
                )
                calculated_crc = modbus_crc16(frame[:14])

                if calculated_crc != expected_crc:
                    # Bad alignment or corrupted frame. Shift one byte
                    # and search again for the next valid header.
                    del buffer[0]
                    continue

                del buffer[:FRAME_SIZE]

                values = struct.unpack(
                    "<6h",
                    frame[2:14],
                )

                wrench = [
                    values[0] / FORCE_SCALE,
                    values[1] / FORCE_SCALE,
                    values[2] / FORCE_SCALE,
                    values[3] / TORQUE_SCALE,
                    values[4] / TORQUE_SCALE,
                    values[5] / TORQUE_SCALE,
                ]

                self._publish_wrench(wrench)

    def _publish_wrench(self, wrench):
        with self.samples_lock:
            self.recent_samples.append(wrench)

        with self.offset_lock:
            offset = list(self.offset)

        corrected = [
            wrench[i] - offset[i]
            for i in range(6)
        ]

        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.wrench.force.x = corrected[0]
        msg.wrench.force.y = corrected[1]
        msg.wrench.force.z = corrected[2]

        msg.wrench.torque.x = corrected[3]
        msg.wrench.torque.y = corrected[4]
        msg.wrench.torque.z = corrected[5]

        self.publisher.publish(msg)

    def zero_callback(self, request, response):
        with self.samples_lock:
            samples = list(self.recent_samples)

        if not samples:
            response.success = False
            response.message = "No F/T samples available yet."
            return response

        new_offset = [
            sum(sample[i] for sample in samples) / len(samples)
            for i in range(6)
        ]

        with self.offset_lock:
            self.offset = new_offset

        response.success = True
        response.message = (
            f"F/T zeroed using {len(samples)} recent samples."
        )

        self.get_logger().info(
            f"{response.message} Offset={new_offset}"
        )

        return response

    def destroy_node(self):
        self.stop_event.set()

        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)

        try:
            self._stop_stream()
        except Exception:
            pass

        if hasattr(self, "serial_port") and self.serial_port.is_open:
            self.serial_port.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = RobotiqFTStreamNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        print(f"Robotiq F/T node error: {exc}")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
