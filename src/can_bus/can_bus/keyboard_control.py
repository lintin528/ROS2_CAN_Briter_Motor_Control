import rclpy
import copy
import curses
import threading
from rclpy.executors import MultiThreadedExecutor
from abc import ABC, abstractmethod
from rclpy.node import Node
from std_msgs.msg import String
from can_bus.can_publisher import *
from can_bus.params import *
from can_bus.utils import *


class State(ABC):
    def __init__(self, tui):
        self.tui = tui

    @abstractmethod
    def handle_input(self, key: int):
        """Handle keyboard input for the current state."""
        pass

    @abstractmethod
    def render(self, stdscr):
        """Render the screen for the current state."""
        pass

class NormalState(State):
    # override. The @override decorator is only supported in Python 3.12+
    def handle_input(self, key: int):
        if key not in KEY_MAP:
            return

        if bool(KEY_MAP[key] & KEY_MAP[curses.KEY_UP]):
            self.tui.select_prev_item()
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_DOWN]):
            self.tui.select_next_item()
        elif bool(KEY_MAP[key] & KEY_MAP[KEY_ENTER]):
            if bool(self.tui.cur_sel & PanelSelect.SPEED):
                self.tui.change_state(SelectWheelState)
            elif bool(self.tui.cur_sel & PanelSelect.POS):
                self.tui.change_state(SelectWheelState)
        elif bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(ExitState)

    # override
    def render(self, stdscr):
        self.tui.display_single(stdscr, STATE_POS, "Panel State: Normal State")
        self.tui.display_menu(stdscr)

class ExitState(State):
    # override
    def handle_input(self, key):
        pass  # No interaction in exit state

    # override
    def render(self, stdscr):
        pass

class SelectWheelState(State):
    # override
    def handle_input(self, key):
        if key not in KEY_MAP:
            return

        if bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(NormalState)
        elif bool(KEY_MAP[key] & KEY_MAP[KEY_ENTER]):
            self.tui.change_state(ControlJointState)
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_UP]):
            self.tui.select_prev_wheel()
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_DOWN]):
            self.tui.select_next_wheel()

    # override
    def render(self, stdscr):
        self.tui.display_single(stdscr, STATE_POS, "Panel State: Select Joint State")
        self.tui.display_menu(stdscr)
        self.tui.display_sel_wheel(stdscr)

class ControlJointState(State):
    # override
    def handle_input(self, key):
        if key not in KEY_MAP:
            return

        if bool(KEY_MAP[key] & KEY_MAP[KEY_ESC]):
            self.tui.change_state(SelectWheelState)
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_UP]):
            self.tui.increase_wheel()
        elif bool(KEY_MAP[key] & KEY_MAP[curses.KEY_DOWN]):
            self.tui.decrease_wheel()

        # Publish data
        if bool(KEY_MAP[key] & (KEY_MAP[curses.KEY_UP] | KEY_MAP[curses.KEY_DOWN])):
            self.tui.can_msg_publisher.publish_can_msg(self.tui.cur_sel_wheel_speed)
            self.tui.msg_cnt += 1

    # override
    def render(self, stdscr):
        self.tui.display_single(stdscr, STATE_POS, "Panel State: Control Joint State")
        self.tui.display_menu(stdscr)
        self.tui.display_control_wheel(stdscr)

class TUI:
    def __init__(self, can_msg_publisher_: CANPublisher):
        self.state: State = NormalState(self)
        self.key: int = 0

        # Arm settings
        self.cur_sel: PanelSelect = PanelSelect.SPEED
        self.cur_sel_wheel: int = 0
        self.cur_speed: list = copy.deepcopy(DEFAULT_FOUR_WHEEL_SPEED)
        self.cur_pos: list = copy.deepcopy(DEFAULT_FOUR_WHEEL_POS)

        self.cur_sel_wheel_speed: list = self.cur_speed
        self.cur_sel_wheel_speed_min: list = MIN_FOUR_WHEEL_SPEED
        self.cur_sel_wheel_speed_max: list = MAX_FOUR_WHEEL_SPEED

        # Parameter settings
        self.step: float = DEFAULT_WHEEL_MOVE_STEP_VEL

        # Publisher for arm control
        self.can_msg_publisher: CANPublisher = can_msg_publisher_
        self.msg_cnt: int = 0

        # Publish the initial joint angles
        self.can_msg_publisher.publish_can_msg(self.cur_sel_wheel_speed)
        self.msg_cnt += 1

    def display_single(self, stdscr, y_position: int, message: str, is_highlight: bool = False):
        style: int = curses.A_REVERSE if is_highlight else 0
        stdscr.addstr(y_position, 0, message, style)

    def display_menu(self, stdscr, y_position: int = MENU_POS):
        self.display_single(
            stdscr, y_position, f"Four wheel speed: {self.cur_speed}", self.cur_sel == PanelSelect.SPEED)
        self.display_single(
            stdscr, y_position + 1, f"Four wheel POS: {self.cur_pos}", self.cur_sel == PanelSelect.POS)

    def display_sel_wheel(self, stdscr, y_position: int = CONTROL_POS):
        # Highlight the active element in the current array
        for i, val in enumerate(self.cur_sel_wheel_speed):
            self.display_single(
                stdscr, y_position + i, f"Wheel {i + 1}: {val}", i == self.cur_sel_wheel)

    def display_control_wheel(self, stdscr, y_position: int = CONTROL_POS):
        # Highlight the active element in the current array
        for i, val in enumerate(self.cur_sel_wheel_speed):
            if i == self.cur_sel_wheel:
                self.display_single(
                    stdscr, y_position + i, f"--> Wheel {i + 1} velocity: {val} <--", True)
            else:
                self.display_single(
                    stdscr, y_position + i, f"Wheel {i + 1} velocity: {val}", False)

    def select_prev_item(self):
        self.cur_sel = prev_bitwise_enum(self.cur_sel, PanelSelect)

    def select_next_item(self):
        self.cur_sel = next_bitwise_enum(self.cur_sel, PanelSelect)

    def change_state(self, new_state_cls: State):
        """Transition to a new state."""
        self.state = new_state_cls(self)

    def select_prev_wheel(self):
        self.cur_sel_wheel = (
            self.cur_sel_wheel - 1) % DEFAULT_WHEEL_NUMBER

    def select_next_wheel(self):
        self.cur_sel_wheel = (
            self.cur_sel_wheel + 1) % DEFAULT_WHEEL_NUMBER
    
    def increase_wheel(self):
        self.cur_sel_wheel_speed[self.cur_sel_wheel] = min(
            self.cur_sel_wheel_speed[self.cur_sel_wheel] + self.step,
            self.cur_sel_wheel_speed_max[self.cur_sel_wheel]
        )
    
    def decrease_wheel(self):
        self.cur_sel_wheel_speed[self.cur_sel_wheel] = max(
            self.cur_sel_wheel_speed[self.cur_sel_wheel] - self.step,
            self.cur_sel_wheel_speed_min[self.cur_sel_wheel]
        )

    def run(self, stdscr):
        curses.curs_set(0)
        stdscr.clear()
        while not isinstance(self.state, ExitState):
            stdscr.clear()
            self.state.render(stdscr)
            stdscr.refresh()
            key = stdscr.getch()
            self.state.handle_input(key)

def curses_main(stdscr, can_msg_publisher: CANPublisher):
    panel = TUI(can_msg_publisher)
    panel.run(stdscr)

def main(args=None):
    rclpy.init(args=args)
    can_msg_publisher = CANPublisher()

    executor = MultiThreadedExecutor()
    executor.add_node(can_msg_publisher)

    spin_thread = threading.Thread(target=rclpy.spin, args=(can_msg_publisher,), daemon=True)
    spin_thread.start()
    try:
        curses.set_escdelay(25)
        curses.wrapper(curses_main, can_msg_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        can_msg_publisher.destroy_node()
        can_msg_publisher.get_logger().info(f'Quit keyboard!')
        rclpy.shutdown()
        spin_thread.join()  # Ensure the spin thread is cleanly stopped

if __name__ == '__main__':
    main()
