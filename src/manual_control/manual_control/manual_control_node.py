#!/usr/bin/env python3
"""
JABARI MARS ROVER - Manual Control Node (Optimized for Vector3 Servo Node)

This revision makes the *minimum optimal* changes needed to interoperate reliably with the
current `Vector3ToPWMNode` servo driver while improving runtime robustness on real hardware.

Key Improvements -------------------------------------------------------------
1. **Latched (Transient Local) Servo Topic**: Ensures the servo node receives the most recent
   Vector3 even if it starts *after* this node. Prevents undefined servo pose at boot.
2. **Optional Periodic Refresh**: Re-publish the last servo angles at a configurable rate so the
   servo node (or downstream tools) re-sync; disabled by default because TRANSIENT_LOCAL handles
   late joiners.
3. **Axis- & Button-Bounds Safety Guards**: Prevent crashes when joystick devices expose fewer
   axes/buttons than expected. Out-of-range mappings are ignored with a warning (once).
4. **Change-Threshold Servo Publishing**: Avoid flooding the bus/I2C + logs on tiny stick noise.
   A configurable degrees threshold controls when an updated Vector3 is sent.
5. **Log Throttling**: Servo and movement logs are rate-limited to cut console spam on the Pi.
6. **3rd Axis Handling**: Still supports fixed-angle third axis (default) or joystick-mapped if
   developer assigns an index. Cleanly handled via helper accessors.

Behavior Compatible With Servo Node ----------------------------------------
- Publishes `geometry_msgs/msg/Vector3` to `/vector3_cmd` (x=pan°, y=tilt°, z=third°).
- Angles remain in degrees [0, 180]; clamped before publish.
- Startup centers servos (90°, 90°, fixed third param) and publishes immediately.

---------------------------------------------------------------------------
PARAMETERS (new + existing)
---------------------------------------------------------------------------
max_linear_velocity (float, default=1.0)
max_angular_velocity (float, default=2.0)
command_timeout (float, default=2.0)
pan_sensitivity_deg (float, default=2.0)
tilt_sensitivity_deg (float, default=2.0)
third_axis_fixed_angle (float, default=90.0)
pivot_cmd_value (float, default=1.0)
speed_step (float, default=0.1)
button_lin_up (int, default=5)
button_lin_down (int, default=4)
button_ang_up (int, default=3)
button_ang_down (int, default=2)
button_toggle_pivot (int, default=7)

# NEW:
servo_refresh_rate_hz (float, default=0.0)
    > If >0, periodically republish last servo angles at this rate.
servo_publish_on_change_deg (float, default=0.5)
    > Minimum degree change required to publish an update (except on force/refresh).
movement_log_period (float, default=0.25)
servo_log_period (float, default=1.0)
    > Minimum seconds between INFO logs for movement & servo state (except force events).
axis_third (int or -1, default=-1)
    > Map a joystick axis to the 3rd servo; if -1 use fixed param.

---------------------------------------------------------------------------
USAGE -----------------------------------------------------------------------
ros2 run <package> manual_control_node

Example YAML override:

manual_control_node:
  ros__parameters:
    pan_sensitivity_deg: 4.0
    tilt_sensitivity_deg: 4.0
    servo_refresh_rate_hz: 5.0
    servo_publish_on_change_deg: 1.0
    axis_third: -1  # keep fixed angle

---------------------------------------------------------------------------
"""

import time
import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy


class ManualControlNode(Node):
    """Manual Control Node for JABARI Rover with joystick, drive modes & servo output."""

    def __init__(self):
        super().__init__('manual_control_node')

        # ------------------------------------------------------------------
        # Parameters (existing)
        # ------------------------------------------------------------------
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('command_timeout', 2.0)
        self.declare_parameter('pan_sensitivity_deg', 2.0)
        self.declare_parameter('tilt_sensitivity_deg', 2.0)
        self.declare_parameter('third_axis_fixed_angle', 90.0)

        # ------------------------------------------------------------------
        # New parameters for drive modes / scaling / button mapping (existing in prior rev)
        # ------------------------------------------------------------------
        self.declare_parameter('pivot_cmd_value', 1.0)   # angular.z to send when pivoting
        self.declare_parameter('speed_step', 0.1)         # increment/decrement step for scales
        self.declare_parameter('button_lin_up', 5)
        self.declare_parameter('button_lin_down', 4)
        self.declare_parameter('button_ang_up', 3)
        self.declare_parameter('button_ang_down', 2)
        self.declare_parameter('button_toggle_pivot', 7)

        # ------------------------------------------------------------------
        # NEW Servo interoperability / robustness parameters
        # ------------------------------------------------------------------
        self.declare_parameter('servo_refresh_rate_hz', 0.0)       # 0.0 = disabled
        self.declare_parameter('servo_publish_on_change_deg', 0.5) # change threshold
        self.declare_parameter('movement_log_period', 0.25)        # seconds between movement logs
        self.declare_parameter('servo_log_period', 1.0)            # seconds between servo logs
        self.declare_parameter('axis_third', -1)                   # -1 => fixed param angle

        # ------------------------------------------------------------------
        # Load params ------------------------------------------------------
        # ------------------------------------------------------------------
        self.max_linear_vel = float(self.get_parameter('max_linear_velocity').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_velocity').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.pan_sensitivity_deg = float(self.get_parameter('pan_sensitivity_deg').value)
        self.tilt_sensitivity_deg = float(self.get_parameter('tilt_sensitivity_deg').value)
        self.third_axis_fixed_angle = float(self.get_parameter('third_axis_fixed_angle').value)
        self.pivot_cmd_value = float(self.get_parameter('pivot_cmd_value').value)
        self.speed_step = float(self.get_parameter('speed_step').value)

        # Button mappings (can be overridden) ------------------------------
        self.BUTTON_LIN_UP = 7             # R1
        self.BUTTON_LIN_DOWN = 6          # L1
        self.BUTTON_ANG_UP = 5             # R2
        self.BUTTON_ANG_DOWN = 4          # L2
        self.BUTTON_TOGGLE_PIVOT_MODE = 0  # Triangle

        # self.BUTTON_LIN_UP = int(self.get_parameter('button_lin_up').value)
        # self.BUTTON_LIN_DOWN = int(self.get_parameter('button_lin_down').value)
        # self.BUTTON_ANG_UP = int(self.get_parameter('button_ang_up').value)
        # self.BUTTON_ANG_DOWN = int(self.get_parameter('button_ang_down').value)
        # self.BUTTON_TOGGLE_PIVOT_MODE = int(self.get_parameter('button_toggle_pivot').value)

        # Servo-related ----------------------------------------------------
        self.servo_refresh_rate_hz = float(self.get_parameter('servo_refresh_rate_hz').value)
        self.servo_publish_on_change_deg = float(self.get_parameter('servo_publish_on_change_deg').value)
        self.movement_log_period = float(self.get_parameter('movement_log_period').value)
        self.servo_log_period = float(self.get_parameter('servo_log_period').value)
        self.AXIS_THIRD_PARAM = int(self.get_parameter('axis_third').value)
        self.AXIS_THIRD: Optional[int] = None if self.AXIS_THIRD_PARAM < 0 else self.AXIS_THIRD_PARAM

        # ------------------------------------------------------------------
        # QoS Profiles
        # ------------------------------------------------------------------
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Transient Local for servo topic so late-joining servo node gets last command.
        servo_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ------------------------------------------------------------------
        # Joystick mapping (axes) -- default Xbox-style; override in code or fork
        # ------------------------------------------------------------------
        # self.BUTTON_ENABLE = 9 
        # self.BUTTON_RESET_PAN_TILT = 8
        # self.AXIS_LINEAR = 1
        # self.AXIS_ANGULAR = 3
        # self.AXIS_PAN = 5
        # self.AXIS_TILT = 6
        # self.AXIS_THIRD set above via param (or None)

        self.BUTTON_ENABLE = 9             # START
        self.BUTTON_RESET_PAN_TILT = 8    # SELECT
        self.AXIS_LINEAR = 1               # Left stick Y
        self.AXIS_ANGULAR = 0              # Left stick X
        self.AXIS_PAN = 2                  # Right stick X
        self.AXIS_TILT = -1                # No right stick Y on UCOM

        # ------------------------------------------------------------------
        # Servo state
        # ------------------------------------------------------------------
        self.pan_angle = 90.0
        self.tilt_angle = 90.0
        self.third_axis_angle = float(self.third_axis_fixed_angle) if self.AXIS_THIRD is None else 90.0

        # Track last-published servo values for change-threshold logic
        self._last_sent_pan = None
        self._last_sent_tilt = None
        self._last_sent_third = None
        self._last_servo_log_time = 0.0

        # Movement log throttle
        self._last_move_log_time = 0.0

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self.vector3_cmd_pub = self.create_publisher(Vector3, '/vector3_cmd', servo_qos)
        self.status_pub = self.create_publisher(String, '/manual_control_status', reliable_qos)

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, reliable_qos)
        self.emergency_stop_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_stop_callback, reliable_qos)

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self.joy_enabled = True #Changed this from false
        self.last_button_states = []
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.emergency_stop_active = False
        self.last_command_time = time.time()
        self.command_count = 0
        self.is_active = True
        self.command_lock = threading.Lock()

        # Drive mode state -------------------------------------------------
        self.pivot_mode = False   # False=Normal, True=Pivot
        self.linear_scale = 1.0   # runtime multiplier on max_linear_vel
        self.angular_scale = 1.0  # runtime multiplier on max_angular_vel

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        self.status_timer = self.create_timer(2.0, self.publish_status)
        self.timeout_timer = self.create_timer(0.1, self.check_command_timeout)

        # Optional periodic servo refresh
        if self.servo_refresh_rate_hz > 0.0:
            period = max(0.01, 1.0 / self.servo_refresh_rate_hz)
            self.servo_refresh_timer = self.create_timer(period, self._servo_refresh_tick)
        else:
            self.servo_refresh_timer = None

        # ------------------------------------------------------------------
        # Startup servo centering (force publish)
        # ------------------------------------------------------------------
        self.publish_vector3_servos(force=True)

        self.get_logger().info(
            'Manual Control Node initialized (Vector3 + Drive Modes + Servo Robustness).\n'
            f'  - /cmd_vel clamp: lin {self.max_linear_vel} m/s, ang {self.max_angular_vel} rad/s\n'
            f'  - Servo topic: /vector3_cmd (x=pan, y=tilt, z=third) [TRANSIENT_LOCAL]\n'
            f'  - 3rd channel mode: {"fixed_param" if self.AXIS_THIRD is None else "joystick"} '
            f'({self.third_axis_fixed_angle}° if fixed)\n'
            f'  - servo_refresh_rate_hz: {self.servo_refresh_rate_hz}\n'
            f'  - publish_on_change_deg: {self.servo_publish_on_change_deg}\n'
            f'  - Timeout: {self.command_timeout}s.\n'
            f'  - Pivot button index: {self.BUTTON_TOGGLE_PIVOT_MODE}\n'
            f'  - Speed step: {self.speed_step}.'
        )

    # ==================================================================
    # Utility: safe axis & button access
    # ==================================================================
    def _axis(self, axes, idx: Optional[int], default: float = 0.0) -> float:
        if idx is None:
            return default
        if idx < 0 or idx >= len(axes):
            self._warn_once(f'Axis index {idx} out of range (len={len(axes)}) - using {default}.')
            return default
        return axes[idx]

    _warned_keys = set()
    def _warn_once(self, key: str):
        if key not in self._warned_keys:
            self._warned_keys.add(key)
            self.get_logger().warn(str(key))

    def _button_rising(self, idx: int, prev_buttons, curr_buttons) -> bool:
        if idx < 0 or idx >= len(curr_buttons) or idx >= len(prev_buttons):
            self._warn_once(f'Button index {idx} out of range (len={len(curr_buttons)}) - ignored.')
            return False
        return (curr_buttons[idx] == 1) and (prev_buttons[idx] == 0)

    # ==================================================================
    # Joystick callback
    # ==================================================================
    def joy_callback(self, msg: Joy):
        # Initialize button state tracking
        if not self.last_button_states:
            self.last_button_states = [0] * len(msg.buttons)

        # Edge-detect mapped buttons -------------------------------------
        if self._button_rising(self.BUTTON_ENABLE, self.last_button_states, msg.buttons):
            self.joy_enabled = not self.joy_enabled
            state = 'ENABLED' if self.joy_enabled else 'DISABLED'
            self.get_logger().info(f'Joystick control {state}')
            if not self.joy_enabled:
                self.send_stop_command()

        if self._button_rising(self.BUTTON_RESET_PAN_TILT, self.last_button_states, msg.buttons):
            self.reset_servos()

        if self._button_rising(self.BUTTON_LIN_UP, self.last_button_states, msg.buttons):
            self.adjust_linear_scale(+self.speed_step)

        if self._button_rising(self.BUTTON_LIN_DOWN, self.last_button_states, msg.buttons):
            self.adjust_linear_scale(-self.speed_step)

        if self._button_rising(self.BUTTON_ANG_UP, self.last_button_states, msg.buttons):
            self.adjust_angular_scale(+self.speed_step)

        if self._button_rising(self.BUTTON_ANG_DOWN, self.last_button_states, msg.buttons):
            self.adjust_angular_scale(-self.speed_step)

        if self._button_rising(self.BUTTON_TOGGLE_PIVOT_MODE, self.last_button_states, msg.buttons):
            self.toggle_pivot_mode()

        self.last_button_states = list(msg.buttons)

        # If enabled and safe, process movement + servos ------------------
        # if self.joy_enabled and not self.emergency_stop_active:
        #     self.handle_movement(msg)
        #     self.handle_servos_from_joystick(msg)
        
        # If enabled and safe, process movement + servos ------------------
        if self.joy_enabled and not self.emergency_stop_active:
            self.handle_movement(msg)
            self.handle_servos_from_joystick(msg)
            # Tilt via X (up) and Box (down) buttons since no right stick Y
            if len(msg.buttons) > 3:
                if msg.buttons[2]:   # X → tilt up
                    self.tilt_angle = self._clamp_angle(self.tilt_angle - self.tilt_sensitivity_deg)
                    self.publish_vector3_servos()
                elif msg.buttons[3]: # Box → tilt down
                    self.tilt_angle = self._clamp_angle(self.tilt_angle + self.tilt_sensitivity_deg)
                    self.publish_vector3_servos()

    # ==================================================================
    # Drive mode + scaling helpers
    # ==================================================================
    def toggle_pivot_mode(self):
        self.pivot_mode = not self.pivot_mode
        self.get_logger().info(f'Drive Mode: {"PIVOT" if self.pivot_mode else "NORMAL"}')

    def adjust_linear_scale(self, delta: float):
        old = self.linear_scale
        self.linear_scale = max(0.0, min(1.0, self.linear_scale + delta))
        self.get_logger().info(f'Linear scale: {self.linear_scale:.1f} (was {old:.1f})')

    def adjust_angular_scale(self, delta: float):
        old = self.angular_scale
        self.angular_scale = max(0.0, min(1.0, self.angular_scale + delta))
        self.get_logger().info(f'Angular scale: {self.angular_scale:.1f} (was {old:.1f})')

    # ==================================================================
    # Movement
    # ==================================================================
    def handle_movement(self, msg: Joy):
        # Raw joystick inputs ------------------------------------------------
        linear_raw = -self._axis(msg.axes, self.AXIS_LINEAR)   # invert so forward stick pushes forward
        angular_raw =  self._axis(msg.axes, self.AXIS_ANGULAR)

        twist = Twist()

        if self.pivot_mode:
            # In pivot mode we ignore linear stick. Use angular stick to choose direction.
            if abs(angular_raw) > 0.2:  # deadzone
                twist.linear.x = 0.0
                twist.angular.z = self.pivot_cmd_value if angular_raw > 0.0 else -self.pivot_cmd_value
            else:
                # no spin request -> stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        else:
            # Normal scaled driving ----------------------------------------
            lin_limit = self.max_linear_vel * self.linear_scale
            ang_limit = self.max_angular_vel * self.angular_scale

            linear = max(-lin_limit, min(lin_limit, linear_raw * lin_limit))
            angular = max(-ang_limit, min(ang_limit, angular_raw * ang_limit))

            twist.linear.x = linear
            twist.angular.z = angular

        # Publish ------------------------------------------------------------
        self.cmd_vel_pub.publish(twist)

        # Update state -------------------------------------------------------
        with self.command_lock:
            self.current_linear_vel = twist.linear.x
            self.current_angular_vel = twist.angular.z
            self.last_command_time = time.time()
            self.command_count += 1

        # Log (throttled) ---------------------------------------------------
        now = time.time()
        if now - self._last_move_log_time >= self.movement_log_period:
            if abs(twist.linear.x) > 0.1 or abs(twist.angular.z) > 0.1:
                if self.pivot_mode:
                    self.get_logger().info(f'PIVOT cmd -> ang={twist.angular.z:+.1f}')
                else:
                    self.get_logger().info(
                        f'Movement: lin={twist.linear.x:.2f} (scale {self.linear_scale:.1f}) '
                        f'ang={twist.angular.z:.2f} (scale {self.angular_scale:.1f})')
                self._last_move_log_time = now

    # ==================================================================
    # Servo handling
    # ==================================================================
    def handle_servos_from_joystick(self, msg: Joy):
        pan_input = self._axis(msg.axes, self.AXIS_PAN) * self.pan_sensitivity_deg
        tilt_input = -self._axis(msg.axes, self.AXIS_TILT) * self.tilt_sensitivity_deg

        if self.AXIS_THIRD is not None:
            third_input = self._axis(msg.axes, self.AXIS_THIRD) * self.tilt_sensitivity_deg
        else:
            third_input = 0.0

        updated = False
        if abs(pan_input) > 0.1:
            self.pan_angle = self._clamp_angle(self.pan_angle + pan_input)
            updated = True
        if abs(tilt_input) > 0.1:
            self.tilt_angle = self._clamp_angle(self.tilt_angle + tilt_input)
            updated = True
        if self.AXIS_THIRD is not None and abs(third_input) > 0.1:
            self.third_axis_angle = self._clamp_angle(self.third_axis_angle + third_input)
            updated = True

        if updated:
            self.publish_vector3_servos()

    def _servo_refresh_tick(self):
        # Periodic republish (non-force, periodic=True so logging throttled)
        self.publish_vector3_servos(periodic=True)

    def publish_vector3_servos(self, force: bool = False, periodic: bool = False):
        # Use fixed param if 3rd axis unmapped
        z_angle = float(self.third_axis_fixed_angle) if self.AXIS_THIRD is None else self.third_axis_angle

        pan = self._clamp_angle(self.pan_angle)
        tilt = self._clamp_angle(self.tilt_angle)
        third = self._clamp_angle(z_angle)

        # Change threshold suppression ------------------------------------
        if not force:
            if (self._last_sent_pan is not None and
                abs(pan - self._last_sent_pan) < self.servo_publish_on_change_deg and
                abs(tilt - self._last_sent_tilt) < self.servo_publish_on_change_deg and
                abs(third - self._last_sent_third) < self.servo_publish_on_change_deg):
                # No meaningful change; skip unless periodic refresh is desired.
                if not periodic:
                    return

        msg = Vector3()
        msg.x = pan
        msg.y = tilt
        msg.z = third
        self.vector3_cmd_pub.publish(msg)

        self._last_sent_pan = pan
        self._last_sent_tilt = tilt
        self._last_sent_third = third

        # Throttled logging ------------------------------------------------
        now = time.time()
        if force or (now - self._last_servo_log_time) >= self.servo_log_period:
            self.get_logger().info(
                f'Servo -> pan={pan:.1f}° tilt={tilt:.1f}° third={third:.1f}°' +
                (' [force]' if force else (' [refresh]' if periodic else ''))
            )
            self._last_servo_log_time = now

    def reset_servos(self):
        self.pan_angle = 90.0
        self.tilt_angle = 90.0
        if self.AXIS_THIRD is None:
            self.third_axis_angle = float(self.third_axis_fixed_angle)
        else:
            self.third_axis_angle = 90.0
        self.publish_vector3_servos(force=True)
        self.get_logger().info('Servos reset to center.')

    # ==================================================================
    # Safety
    # ==================================================================
    def send_stop_command(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        with self.command_lock:
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
        self.get_logger().info('Stop command sent.')

    def emergency_stop_callback(self, msg: Bool):
        self.emergency_stop_active = msg.data
        if self.emergency_stop_active:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            self.send_stop_command()
            self.joy_enabled = False
        else:
            self.get_logger().info('Emergency stop cleared.')

    def check_command_timeout(self):
        current_time = time.time()
        with self.command_lock:
            time_since_last = current_time - self.last_command_time
            if time_since_last > self.command_timeout and (self.current_linear_vel != 0.0 or self.current_angular_vel != 0.0):
                self.get_logger().warn(f'Command timeout ({time_since_last:.1f}s) - stopping rover.')
                self.send_stop_command()

    # ==================================================================
    # Status
    # ==================================================================
    def publish_status(self):
        try:
            with self.command_lock:
                status_data = {
                    'timestamp': time.time(),
                    'node_active': self.is_active,
                    'joystick_enabled': self.joy_enabled,
                    'emergency_stop_active': self.emergency_stop_active,
                    'drive_mode': 'pivot' if self.pivot_mode else 'normal',
                    'speed_scales': {
                        'linear_scale': self.linear_scale,
                        'angular_scale': self.angular_scale,
                        'speed_step': self.speed_step,
                    },
                    'movement': {
                        'linear_velocity': self.current_linear_vel,
                        'angular_velocity': self.current_angular_vel,
                        'time_since_last_command': time.time() - self.last_command_time,
                    },
                    'servos': {
                        'pan_angle': self.pan_angle,
                        'tilt_angle': self.tilt_angle,
                        'third_angle': (self.third_axis_angle if self.AXIS_THIRD is not None else self.third_axis_fixed_angle),
                        'third_axis_mode': 'joystick' if self.AXIS_THIRD is not None else 'fixed_param',
                    },
                    'stats': {
                        'total_commands': self.command_count,
                        'max_linear_vel': self.max_linear_vel,
                        'max_angular_vel': self.max_angular_vel,
                    },
                }

            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')

    # ==================================================================
    # Shutdown
    # ==================================================================
    def shutdown_gracefully(self):
        self.get_logger().info('Shutting down Manual Control Node...')
        self.send_stop_command()
        self.reset_servos()
        self.is_active = False
        self.publish_status()
        self.get_logger().info('Manual Control Node shutdown complete.')

    # ==================================================================
    # Helpers
    # ==================================================================
    @staticmethod
    def _clamp_angle(angle: float) -> float:
        return max(0.0, min(180.0, angle))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ManualControlNode()
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        node.get_logger().info('Manual Control Node (optimized) running...')
        executor.spin()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Keyboard interrupt.')
    except Exception as e:  # pylint: disable=broad-except
        print(f'Error starting manual control node: {e}')
    finally:
        if node is not None:
            node.shutdown_gracefully()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
