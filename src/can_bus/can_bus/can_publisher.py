import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor

class CANPublisher(Node):
    def __init__(self):
        super().__init__('CAN_Publisher')
        self.topic_name = 'four_wheel_speed'
        self._four_wheel_speed_publisher = self.create_publisher(Float32MultiArray, self.topic_name, 10)

    def publish_can_msg(self, four_wheel_speed_msg:list):
        msg = Float32MultiArray()
        msg.data = four_wheel_speed_msg
        self._four_wheel_speed_publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    try:
        # Create the node
        wheel_publisher = CANPublisher()

        # Use a MultiThreadedExecutor to support threading in nodes
        executor = MultiThreadedExecutor()
        executor.add_node(wheel_publisher)

        # Start the executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
    except KeyboardInterrupt:
        pass
    finally:
        # Shut down the node and executor properly
        wheel_publisher.destroy_node()
        executor.shutdown()
        executor_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
