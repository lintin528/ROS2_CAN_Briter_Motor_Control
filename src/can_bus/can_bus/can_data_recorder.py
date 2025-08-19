import rclpy
import time
import threading
import serial
import copy
import numpy as np
from scipy.interpolate import CubicSpline
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
        self.declare_parameter("mode", "speed")
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.get_logger().info(f"Using mode: {self.mode}")

        self.get_logger().info(f'recorder start!')
        self.record_period = 15
        self.target_speed = [1000.0, 0.0, 0.0, 0.0]
        self.init_speed = copy.deepcopy(DEFAULT_FOUR_WHEEL_SPEED)

        self.target_pos = copy.deepcopy(DEFAULT_FOUR_WHEEL_POS)
        self.init_pos = copy.deepcopy(DEFAULT_FOUR_WHEEL_POS)

        self.can_msg_publisher = _can_msg_publisher
        self.can_reader = _can_reader
        self.counter = 0

        self.dt = 0.05
        self.total_time = 10
        self.time_array = np.arange(0, self.total_time, self.dt)

        self.get_input = {
        'step': self.step_input,
        'three_stage_ramp': self.three_stage_ramp,
        'sin_cos': self.sin_cos_input,
        'cubic': self.cubicSpline_input,
        }

        if self.mode == "pos":
            self._timer = self.create_timer(self.record_period, self.timer_callback_pos)
        else:
            self._timer = self.create_timer(self.record_period, self.timer_callback)

    def step_input(self, time, step_time=0.02, amplitude=1):
        """ generate (Step) input signal """
        input_signal = np.zeros_like(time)
        input_signal[time >= step_time] = amplitude
        return input_signal

    def three_stage_ramp(self, time, t_peak1, t_peak2, slope_up1=1, slope_down=-1, slope_up2=0.5):
        """ generate (Ramp) input signal """
        y = np.piecewise(time,
            [time < t_peak1, 
            (time >= t_peak1) & (time < t_peak2), 
            time >= t_peak2],
            [lambda t: slope_up1 * t,  
            lambda t: slope_down * (t - t_peak1) + slope_up1 * t_peak1,  
            lambda t: slope_up2 * (t - t_peak2) + (slope_down * (t_peak2 - t_peak1) + slope_up1 * t_peak1)]
        )
        return y

    def sin_cos_input(self, time, A_sin=1.0, A_cos=0.5, f_sin=1.0, f_cos=1.0, phase_sin=0.0, phase_cos=0.0):
        y = A_sin * np.sin(2 * np.pi * f_sin * time + phase_sin) + A_cos * np.cos(2 * np.pi * f_cos * time + phase_cos)
        return y

    def cubicSpline_input(self, time):
        num_control_points = np.random.randint(5, 10)  # 隨機取 5~10 個控制點
        control_times = np.sort(np.random.choice(time, num_control_points, replace=False))
        control_values = np.random.uniform(-5, 5, num_control_points)  # 隨機馬達輸入變化
        
        spline = CubicSpline(control_times, control_values)
        return spline(time)

    def get_random_ramp_input(self):
        """ Generate random ramp input signal """
        t_peak1 = np.random.uniform(2, 4)  # Random peak time
        t_peak2 = np.random.uniform(6, 8)
        slope_up1 = np.random.uniform(200000, 320000)
        slope_down = np.random.uniform(-360000, -200000)
        slope_up2 = np.random.uniform(200000, 320000)
        return self.get_input['three_stage_ramp'](self.time_array, t_peak1, t_peak2, slope_up1, slope_down, slope_up2)

    def get_random_sin_cos_input(self):
        """ Generate random sin + cos input signal """
        A_sin = np.random.uniform(150000, 250000)
        A_cos = np.random.uniform(150000, 250000)
        frequencies = [0.1, 0.2, 0.4]

        f_sin = np.random.choice(frequencies)
        f_cos = np.random.choice(frequencies)
        phase_sin = np.random.uniform(0, 2 * np.pi)
        phase_cos = np.random.uniform(0, 2 * np.pi)
        return self.get_input['sin_cos'](self.time_array, A_sin, A_cos, f_sin, f_cos, phase_sin, phase_cos)

    def constant_input(self, value=160000.0):
        return np.full_like(self.time_array, value)

    def timer_callback(self):
        self.get_logger().info(f'collect start num {self.counter}')
        self.counter += 1
        self.can_msg_publisher.publish_can_msg(self.init_speed)
        self.can_reader.target_value = copy.deepcopy(self.init_speed)
        time.sleep(1.0)
        threading.Thread(
                target=self.can_reader.send_encoder_inquiry,
                args=(ReadModeSelect.SPEED,),
                daemon=False
            ).start()
        time.sleep(1.0)
        self.can_reader.target_value = copy.deepcopy(self.target_speed)
        self.can_msg_publisher.publish_can_msg(self.target_speed)

    def timer_callback_pos(self):
        self.get_logger().info(f'collect start num {self.counter}')
        self.counter += 1
        self.can_msg_publisher.publish_can_msg(self.init_speed)
        self.can_msg_publisher.publish_can_msg_pos(self.init_pos)
        self.can_reader.target_value = copy.deepcopy(self.init_speed)
        self.can_reader.target_value_pos = copy.deepcopy(self.init_pos[0])
        self.can_reader.init_motor_pos()
        time.sleep(2.0)
        threading.Thread(
                target=self.can_reader.send_encoder_inquiry,
                args=(ReadModeSelect.SPEED,),
                daemon=False
            ).start()
        threading.Thread(
                target=self.can_reader.send_encoder_inquiry,
                args=(ReadModeSelect.POS,),
                daemon=False
            ).start()
        time.sleep(1.0)
        # --------- # Set position # --------- #
        start_time = time.time()
        # input_signal = self.get_random_ramp_input()
        input_signal = self.get_random_sin_cos_input()
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time

            if elapsed_time > self.total_time:
                break

            index = int(np.clip(round(elapsed_time / self.dt), 0, len(input_signal) - 1))
            self.can_reader.target_value_pos = copy.deepcopy(input_signal[index])
            self.can_msg_publisher.publish_can_msg_pos([input_signal[index]])

            time.sleep(self.dt*10)
        # --------- # Set position # --------- #
def main(args=None):
    rclpy.init(args=args)

    try:
        # Create the node
        can_msg_publisher = CANPublisher()
        can_reader = CANReader()
        
        # Use a MultiThreadedExecutor to support threading in nodes
        executor = MultiThreadedExecutor()
        executor.add_node(can_msg_publisher)
        executor.add_node(can_reader)
        
        # Start the executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        can_recorder = CANRecorder(can_msg_publisher, can_reader)
        executor.add_node(can_recorder)
        # rclpy.spin(can_recorder)
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
