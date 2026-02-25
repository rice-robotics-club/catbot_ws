'''
Xbox controller command node. Reads joystick input and publishes Twist commands.
Falls back to keyboard (WASD / arrow keys) if no controller is connected.
Falls back to zero commands if neither stdin is a tty nor pygame is available.

Axis mapping (Linux xpad driver, Xbox controller):
  Left stick vertical  (axis 1): up=-1 → negate → linear_x  (forward/back)
  Right stick horiz.   (axis 3): left=-1 → negate → angular_z (turn left/right)

Keyboard mapping (fallback):
  W / ↑ : forward         S / ↓ : backward
  A / ← : turn left       D / → : turn right
  Space / Q : stop

Deadzone of 0.1 applied to ignore small stick drift.
'''

import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Allow pygame to run without a display (headless / robot environment)
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
os.environ.setdefault('SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS', '1')

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

DEADZONE = 0.1
LINEAR_SPEED = 1.0
ANGULAR_SPEED = 1.0
# If no key arrives within this window (seconds), velocity is zeroed (key released)
KEY_TIMEOUT = 0.05  # select() wait window — short poll so the loop stays responsive
KEY_DECAY   = 0.5  # zero velocity when no keypress received for this long (must exceed
                    # OS initial key-repeat delay, which is ~500 ms on macOS by default)


def _apply_deadzone(value: float) -> float:
    return value if abs(value) >= DEADZONE else 0.0


class KeyboardReader:
    """Non-blocking keyboard reader that runs in a background thread using raw terminal mode."""

    _ARROW_UP    = '\x1b[A'
    _ARROW_DOWN  = '\x1b[B'
    _ARROW_RIGHT = '\x1b[C'
    _ARROW_LEFT  = '\x1b[D'

    def __init__(self):
        self.linear_x = 0.0
        self.angular_z = 0.0
        self._last_key_time = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._old_settings = None

    def start(self) -> bool:
        """Start the reader thread. Returns False if stdin is not a tty."""
        if not sys.stdin.isatty():
            return False
        self._old_settings = termios.tcgetattr(sys.stdin)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._running = False
        if self._old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

    def get_cmd(self) -> tuple[float, float]:
        with self._lock:
            # Return zero transiently if no key has arrived recently — do NOT
            # mutate stored values so that a late key-repeat immediately
            # restores velocity without a flicker cycle.
            if time.monotonic() - self._last_key_time > KEY_DECAY:
                return 0.0, 0.0
            return self.linear_x, self.angular_z

    def _read_loop(self):
        fd = sys.stdin.fileno()
        tty.setraw(fd)
        try:
            while self._running:
                ready, _, _ = select.select([sys.stdin], [], [], KEY_TIMEOUT)
                if not ready:
                    continue

                # os.read() bypasses Python's BufferedReader entirely.
                # sys.stdin.read(1) would pull many bytes into Python's internal
                # buffer in one syscall, leaving the fd empty so subsequent
                # select() calls returned "not ready" — causing key-repeat events
                # to be silently dropped and _last_key_time to go stale.
                # Reading up to 256 bytes and parsing all of them in one pass
                # ensures every pending key-repeat stamps _last_key_time.
                raw = os.read(fd, 256)
                i = 0
                while i < len(raw):
                    byte = raw[i]

                    # Arrow key escape sequence: \x1b [ A/B/C/D  (3 bytes)
                    if byte == 0x1b and i + 2 < len(raw) and raw[i + 1] == ord('['):
                        ch = {ord('A'): self._ARROW_UP,
                              ord('B'): self._ARROW_DOWN,
                              ord('C'): self._ARROW_RIGHT,
                              ord('D'): self._ARROW_LEFT}.get(raw[i + 2])
                        i += 3
                    elif byte == 0x1b:
                        ch = None  # lone ESC or partial sequence — skip
                        i += 1
                    else:
                        ch = chr(byte)
                        i += 1

                    if ch is None:
                        continue

                    with self._lock:
                        if ch in ('w', 'W', self._ARROW_UP):
                            self.linear_x = LINEAR_SPEED
                            self.angular_z = 0.0
                            self._last_key_time = time.monotonic()
                        elif ch in ('s', 'S', self._ARROW_DOWN):
                            self.linear_x = -LINEAR_SPEED
                            self.angular_z = 0.0
                            self._last_key_time = time.monotonic()
                        elif ch in ('a', 'A', self._ARROW_LEFT):
                            self.linear_x = 0.0
                            self.angular_z = ANGULAR_SPEED
                            self._last_key_time = time.monotonic()
                        elif ch in ('d', 'D', self._ARROW_RIGHT):
                            self.linear_x = 0.0
                            self.angular_z = -ANGULAR_SPEED
                            self._last_key_time = time.monotonic()
                        elif ch in (' ', 'q', 'Q'):
                            self.linear_x = 0.0
                            self.angular_z = 0.0
                            self._last_key_time = time.monotonic()
                        elif ch == '\x03':  # Ctrl+C — stop thread cleanly
                            self._running = False
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)


class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        self.cmd_pub = self.create_publisher(Twist, 'commands/data', 10)
        self.joystick = None
        self._keyboard = None
        self._init_joystick()
        self.timer = self.create_timer(0.1, self.publish_cmd)

    # ------------------------------------------------------------------
    # Joystick initialisation
    # ------------------------------------------------------------------

    def _init_joystick(self):
        if not PYGAME_AVAILABLE:
            self.get_logger().warn('pygame not installed — falling back to keyboard (WASD)')
            self._start_keyboard()
            return

        pygame.init()
        pygame.joystick.init()

        if not self._try_connect_joystick():
            self.get_logger().warn('No controller found — falling back to keyboard (WASD)')
            self._start_keyboard()

    def _try_connect_joystick(self) -> bool:
        pygame.joystick.quit()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            return False

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f'Controller connected: {self.joystick.get_name()}')
        return True

    # ------------------------------------------------------------------
    # Keyboard fallback
    # ------------------------------------------------------------------

    def _start_keyboard(self):
        self._keyboard = KeyboardReader()
        if self._keyboard.start():
            self.get_logger().info(
                'Keyboard control active — focus THIS terminal and use: '
                'W/S or ↑/↓: forward/back  |  A/D or ←/→: turn  |  Space: stop'
            )
        else:
            self.get_logger().warn('stdin is not a tty — publishing zero commands')
            self._keyboard = None

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def publish_cmd(self):
        cmd = Twist()

        if PYGAME_AVAILABLE:
            pygame.event.pump()

            if self.joystick is None:
                self._try_connect_joystick()
            else:
                try:
                    if not self.joystick.get_init():
                        self.get_logger().warn('Controller disconnected — attempting reconnect')
                        self.joystick = None
                except Exception:
                    self.get_logger().warn('Controller error — attempting reconnect')
                    self.joystick = None

        if self.joystick is not None:
            # Left stick vertical: up = -1, so negate for forward = positive
            cmd.linear.x = _apply_deadzone(-self.joystick.get_axis(1))
            # Right stick horizontal: left = -1, so negate for left turn = positive angular_z
            cmd.angular.z = _apply_deadzone(-self.joystick.get_axis(3))
        elif self._keyboard is not None:
            cmd.linear.x, cmd.angular.z = self._keyboard.get_cmd()

        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        if self._keyboard is not None:
            self._keyboard.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    command_node = CommandNode()
    rclpy.spin(command_node)
    command_node.destroy_node()
    rclpy.shutdown()
