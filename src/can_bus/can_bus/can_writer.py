import rclpy
import time
import threading
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor
import serial

class CANWriter(Node):
    def __init__(self):
        super().__init__('can_subscriber')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'four_wheel_speed',
            self._keyboard_callback,
            10
        )

        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=2000000,
            timeout=0.1
        )
        self.subscription
        self.init_can_module()

        # try:
        #     self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        #     self.get_logger().info("CAN bus initialized.")
        # except Exception as e:
        #     self.get_logger().error(f"CAN init failed: {e}")
        #     self.bus = None

    def init_can_module(self):
        frame = [
            0xaa, 0x55, 0x12,
            0x03,  # CAN speed: 500k = 0x03
            0x01,  # Standard frame
            0x00, 0x00, 0x00, 0x00,  # Filter ID
            0x00, 0x00, 0x00, 0x00,  # Mask ID
            0x00,  # Normal mode
            0x01, 0x00, 0x00, 0x00, 0x00  # Reserved
        ]
        checksum = sum(frame[2:19]) & 0xFF
        frame.append(checksum)
        self.ser.write(bytearray(frame))
        self.get_logger().info('CAN adaptor configured')

    def _keyboard_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 1:
            self.get_logger().warn('Empty CAN message received')
            return

        can_id = msg.data[0]
        data = msg.data[1:]

        if len(data) > 8:
            self.get_logger().warn('Too much CAN data, truncating to 8 bytes')
            data = data[:8]

        frame = bytearray()
        frame.append(0xAA)  # Start byte

        info_byte = 0xC0
        info_byte |= len(data)  # DLC
        frame.append(info_byte)

        # ID (2 bytes)
        frame.append(can_id & 0xFF)     # LSB
        frame.append((can_id >> 8) & 0xFF)  # MSB

        # Data
        frame.extend(data)

        frame.append(0x55)  # End byte

        self.ser.write(frame)

        # Debug print
        hex_str = ' '.join(f'{b:02X}' for b in frame)
        self.get_logger().info(f'Sent CAN frame: {hex_str}')

        # for i, val in enumerate(msg.data):
        #     try:
        #         speed_bytes = int(val).to_bytes(4, byteorder='big', signed=True)
        #         command_flag = bytes([0x02]) # Command flag for setting speed
        #         data_bytes = command_flag + speed_bytes
        #         can_msg = can.Message(arbitration_id=0x00 + i,
        #                               data=data_bytes,
        #                               is_extended_id=False)
        #         self.bus.send(can_msg)
        #         self.get_logger().info(f"Sent CAN msg ID {hex(0x00 + i)}: {data_bytes.hex()}")
        #     except Exception as e:
        #         self.get_logger().error(f"Failed to send CAN msg: {e}")


def main(args=None):
    rclpy.init(args=args)

    try:
        # Create the node
        can_writer = CANWriter()

        # Use a MultiThreadedExecutor to support threading in nodes
        executor = MultiThreadedExecutor()
        executor.add_node(can_writer)

        # Start the executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Shut down the node and executor properly
        can_writer.destroy_node()
        executor.shutdown()
        executor_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
