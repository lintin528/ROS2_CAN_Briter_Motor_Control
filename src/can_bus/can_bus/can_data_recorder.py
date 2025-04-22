import rclpy
import time
import threading
import serial
import copy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor
from can_bus.params import *
from can_bus.utils import *
from can_bus.can_reader import *
from can_bus.can_publisher import *


class CANRecorder(Node):
    def __init__(self, _can_msg_publisher: CANPublisher, _can_reader: CANReader):
        super().__init__('can_recorder')
        self.record_period = 15
        self.target_speed = [10, 0, 0, 0]
        self.init_speed = copy.deepcopy(DEFAULT_FOUR_WHEEL_SPEED)
        self.can_msg_publisher = _can_msg_publisher
        self.can_reader = can_reader
        self._timer = self.create_timer(self.record_period, self.timer_callback)

    def timer_callback(self):
        self.can_msg_publisher.publish_can_msg(self.init_speed)
        time.sleep(1.0)
        threading.Thread(
            target=self.can_reader.send_encoder_inquiry,
            args=(ReadModeSelect.SPEED,),
            daemon=False
        ).start()
        time.sleep(1.0)
        self.can_reader.target_value = copy.deepcopy(self.target_speed)
        self.can_msg_publisher.publish_can_msg(self.target_speed)
            
def main(args=None):
    rclpy.init(args=args)

    try:
        # Create the node
        can_msg_publisher = CANPublisher()
        can_reader = CANReader()
        can_recorder = CANRecorder(can_msg_publisher, can_reader)
        # Use a MultiThreadedExecutor to support threading in nodes
        executor = MultiThreadedExecutor()
        executor.add_node(can_msg_publisher)
        executor.add_node(can_reader)
        executor.add_node(can_recorder)

        # Start the executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Shut down the node and executor properly
        can_msg_publisher.destroy_node()
        can_reader.destroy_node()
        can_recorder.destroy_node()
        executor.shutdown()
        executor_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
