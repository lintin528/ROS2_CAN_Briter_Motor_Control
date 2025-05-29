import rclpy
import time
import threading
import serial
import copy
import os
import csv
import datetime
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor
from can_bus.params import *
from can_bus.utils import *


class CANReader(Node):
    def __init__(self):
        super().__init__('CAN_Reader')
        self.get_logger().info(f'reader start!')
        self.encoder_cur_speed: list = copy.deepcopy(DEFAULT_FOUR_WHEEL_SPEED)
        self.encoder_cur_pos: list = copy.deepcopy(DEFAULT_FOUR_WHEEL_POS)
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=CAN_BAUDRATE,
            timeout=CAN_TIMEOUT
        )
        self.folder_path = './data_pos'
        self.timestamp_str = ""
        self.file_name = ""
        
        self.start_time = time.time()
        self.target_value = [0.0] * len(WHEEL_CAN_IDS)
        self.target_value_pos = 0.0
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)
        self.return_ser = ""
        self.start_read_thread()

    def read_can_response(self):
        while self.ser.in_waiting:
            header = self.ser.read(1)
            if header != b'\xAA':
                continue

            info = self.ser.read(1)
            dlc = info[0] & 0x0F  # DLC
            is_standard = not (info[0] & 0x20)

            id_lsb = self.ser.read(1)[0]
            id_msb = self.ser.read(1)[0]
            can_id = (id_msb << 8) | id_lsb

            byte_val = list(self.ser.read(dlc))
            speed_val = int.from_bytes(byte_val[2:6], byteorder='big', signed=True)
            footer = self.ser.read(1)

            type_id = (byte_val[0] << 8) | byte_val[1]
            timestamp = time.time() - self.start_time
            try:
                index = max(can_id - 1, 0)
                target = self.target_value[index]
                if type_id == 0x0F01:  # 速度
                    speed_val = int.from_bytes(byte_val[2:6], byteorder='big', signed=True)
                    self.encoder_cur_speed[index] = speed_val
                    print(f"[SPEED] type=0x{type_id:X}, speed={speed_val}, time={timestamp:.3f}s, target={target}")
                    if speed_val < 2000:
                        with open(self.file_name, mode='a', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow([timestamp, speed_val, target])

                elif type_id == 0x0F08:  # 位置
                    pos_val = int.from_bytes(byte_val[2:6], byteorder='big', signed=True)
                    self.encoder_cur_pos[index] = pos_val
                    print(f"[POSITION] type=0x{type_id:X}, pos={pos_val}, time={timestamp:.3f}s, target={target}")
                    with open("position.csv", mode='a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([timestamp, pos_val, target])

                # self.return_ser = (
                #     f"[RAW] header={header.hex()} info={info.hex()} id=0x{can_id:X} data={byte_val} "
                #     f"footer={footer.hex()} | time={timestamp:.3f}s | speed={speed_val} | target={target} | index={can_id-1}"
                # )
                # print(self.return_ser)

            except IndexError:
                self.get_logger().error(f"CAN ID {can_id} is out of range for encoder_cur_speed list.")
                pass

    def send_encoder_inquiry(self, mode: ReadModeSelect):
        self.timestamp_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.file_name = f"{self.folder_path}/data_{self.timestamp_str}.csv"
        self.file = open(self.file_name, mode='w', newline='')
        with open(self.file_name, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Encoder Position', 'Target Position'])
            print(f"Data will be saved to {self.file_name}")

        for t in range(int(CAN_INQUIRY_DURATION / CAN_INQUIRY_FREQUENCY)):
            for i in range(len(WHEEL_CAN_IDS)):
                can_id = WHEEL_CAN_IDS[i]

                if mode == ReadModeSelect.SPEED:
                    ins_bytes = bytes(CAN_INSTRUCTION_BYTE_INQUIRY_DICT['CAN_INSTRUCTION_BYTE_INQUIRY_SPEED'])
                elif mode == ReadModeSelect.POS:
                    ins_bytes = bytes(CAN_INSTRUCTION_BYTE_INQUIRY_DICT['CAN_INSTRUCTION_BYTE_INQUIRY_POS'])

                frame = bytearray()
                frame.append(0xAA)  # Start byte

                info_byte = 0xC0
                info_byte |= len(ins_bytes)  # DLC
                frame.append(info_byte)

                # ID (2 bytes)
                frame.append(can_id & 0xFF)     # LSB
                frame.append((can_id >> 8) & 0xFF)  # MSB

                frame.extend(ins_bytes)
                frame.append(0x55)

                self.ser.write(frame)
            time.sleep(CAN_INQUIRY_FREQUENCY)
    
    def start_read_thread(self):
        self.read_thread = threading.Thread(target=self.read_loop, daemon=False)
        self.read_thread.start()

    def read_loop(self):
        while rclpy.ok():
            self.read_can_response()
            time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)

    try:
        # Create the node
        can_reader = CANReader()

        # Use a MultiThreadedExecutor to support threading in nodes
        executor = MultiThreadedExecutor()
        executor.add_node(can_reader)

        # Start the executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Shut down the node and executor properly
        can_reader.destroy_node()
        executor.shutdown()
        executor_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
