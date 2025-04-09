import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor
import can

class CANWriter(Node):
    def __init__(self):
        super().__init__('can_subscriber')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'four_wheel_speed',
            self._keyboard_callback,
            10
        )
        self.subscription

        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.get_logger().info("CAN bus initialized.")
        except Exception as e:
            self.get_logger().error(f"CAN init failed: {e}")
            self.bus = None

    def _keyboard_callback(self, msg: Float32MultiArray):
        if self.bus is None:
            self.get_logger().warn("CAN bus not initialized.")
            return

        for i, val in enumerate(msg.data):
            try:
                speed_bytes = int(val).to_bytes(4, byteorder='big', signed=True)
                command_flag = bytes([0x02]) # Command flag for setting speed

                data_bytes = command_flag + speed_bytes
                can_msg = can.Message(arbitration_id=0x00 + i,
                                      data=data_bytes,
                                      is_extended_id=False)
                self.bus.send(can_msg)
                self.get_logger().info(f"Sent CAN msg ID {hex(0x100 + i)}: {data_bytes}")
            except Exception as e:
                self.get_logger().error(f"Failed to send CAN msg: {e}")


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
