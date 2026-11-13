"""
Joystick to MAVLink — owns serial port, forwards to MAVROS via UDP.

This node directly controls the FC serial port and forwards all FC
telemetry to a UDP port that MAVROS reads for ROS topic publishing.

Architecture:
  /dev/ttyACM0 ←→ this node ←→ UDP:14550 → MAVROS (read-only)
                      ↑
              /joy topic from topside
"""

import threading
import socket
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn, State, VfrHud
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Float64, Bool, String
from pymavlink import mavutil

STICK_DEADZONE = 0.08
NOISE_THRESHOLD = 0.03
CHAN_NOCHANGE = 65535


def apply_deadzone(value, deadzone=STICK_DEADZONE):
    if abs(value) < deadzone: return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


class PIDController:
    def __init__(self, kp=1.0, ki=0.1, kd=0.5, output_limit=1.0):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.output_limit = output_limit
        self.setpoint = 0.0; self.integral = 0.0
        self.prev_error = 0.0; self.prev_time = None
        self.integral_limit = output_limit / max(ki, 0.001)

    def reset(self):
        self.integral = 0.0; self.prev_error = 0.0; self.prev_time = None

    def set_gains(self, kp, ki, kd):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.integral_limit = self.output_limit / max(ki, 0.001)

    def compute(self, current, t):
        err = self.setpoint - current
        if self.prev_time is None:
            self.prev_time = t; self.prev_error = err; return 0.0
        dt = t - self.prev_time
        if dt <= 0: return 0.0
        self.integral = max(-self.integral_limit,
            min(self.integral_limit, self.integral + err * dt))
        out = self.kp*err + self.ki*self.integral + self.kd*(err-self.prev_error)/dt
        self.prev_error = err; self.prev_time = t
        return max(-self.output_limit, min(self.output_limit, out))


class RelayAutoTuner:
    """
    Astrom-Hagglund relay auto-tuner.

    Replaces the controller with a relay (bang-bang) that switches output
    between +d and -d based on the sign of error. The system then
    oscillates at its critical frequency with period Tu and amplitude a.
    From these the ultimate gain Ku is computed:
        Ku = 4 * d / (pi * a)
    Then Ziegler-Nichols rules give PID gains:
        Kp = 0.6 * Ku
        Ki = 1.2 * Ku / Tu  (i.e. Ti = Tu / 2)
        Kd = 0.075 * Ku * Tu (i.e. Td = Tu / 8)
    """

    def __init__(self, relay_amplitude=0.4, hysteresis=0.05,
                 min_cycles=4, timeout=60.0):
        self.d = relay_amplitude
        self.h = hysteresis  # error band to suppress noise-driven flips
        self.min_cycles = min_cycles
        self.timeout = timeout

        self.active = False
        self.setpoint = 0.0
        self.relay_state = +1  # +1 or -1
        self.start_time = None

        # Track switches and peaks for period + amplitude calculation
        self.switch_times = []  # times of relay sign flips
        self.peak_high = -1e9
        self.peak_low = +1e9
        self.peak_history = []  # list of (time, peak_high, peak_low) per cycle

    def start(self, setpoint, t):
        self.active = True
        self.setpoint = setpoint
        self.relay_state = +1
        self.start_time = t
        self.switch_times = []
        self.peak_high = -1e9
        self.peak_low = +1e9
        self.peak_history = []

    def stop(self):
        self.active = False

    def step(self, current, t):
        """
        Returns (relay_output, done, gains_dict_or_None).
        relay_output is in [-d, +d]; gains_dict has kp, ki, kd, ku, tu when done.
        """
        if not self.active:
            return 0.0, False, None

        if t - self.start_time > self.timeout:
            self.active = False
            return 0.0, True, None  # timeout, no gains

        error = self.setpoint - current

        # Track local extremes within the current half-cycle
        if self.relay_state > 0:
            # output is +d, system should drive UP, peak_high tracks max
            if current > self.peak_high:
                self.peak_high = current
        else:
            if current < self.peak_low:
                self.peak_low = current

        # Hysteresis-aware relay switching
        switched = False
        if self.relay_state > 0 and error < -self.h:
            # process overshot, flip to -d
            self.relay_state = -1
            switched = True
        elif self.relay_state < 0 and error > self.h:
            self.relay_state = +1
            switched = True

        if switched:
            self.switch_times.append(t)
            # Record peak from the half-cycle that just ended
            if len(self.switch_times) >= 2:
                self.peak_history.append((t, self.peak_high, self.peak_low))
            # Reset peak trackers for the new half-cycle
            self.peak_high = current
            self.peak_low = current

        # Need at least min_cycles full oscillations to compute gains
        # Each full cycle = 2 switches, so we need 2*min_cycles + 1 switches
        if len(self.switch_times) >= 2 * self.min_cycles + 1:
            return self._finish()

        return self.relay_state * self.d, False, None

    def _finish(self):
        """Compute Ku, Tu, and PID gains from oscillation data."""
        import math

        # Average period from the last few full cycles
        # A full period spans 2 switches (one + cycle, one - cycle)
        switches = self.switch_times
        # Period = time between every-other switch
        periods = []
        for i in range(2, len(switches)):
            periods.append(switches[i] - switches[i - 2])
        if not periods:
            self.active = False
            return 0.0, True, None
        tu = sum(periods[-min(4, len(periods)):]) / min(4, len(periods))

        # Amplitude = average peak-to-peak / 2 from the last few cycles
        recent = self.peak_history[-min(4, len(self.peak_history)):]
        if not recent:
            self.active = False
            return 0.0, True, None
        amps = [(p[1] - p[2]) / 2.0 for p in recent if p[1] > p[2]]
        if not amps:
            self.active = False
            return 0.0, True, None
        a = sum(amps) / len(amps)
        if a < 1e-6:
            self.active = False
            return 0.0, True, None

        # Astrom-Hagglund critical gain
        ku = (4.0 * self.d) / (math.pi * a)

        # Ziegler-Nichols classic PID rule
        kp = 0.6 * ku
        ti = tu / 2.0
        td = tu / 8.0
        ki = kp / ti if ti > 1e-6 else 0.0
        kd = kp * td

        # Sanity clamp — anything beyond these is probably nonsense
        kp = max(0.05, min(5.0, kp))
        ki = max(0.005, min(2.0, ki))
        kd = max(0.005, min(3.0, kd))

        self.active = False
        return 0.0, True, {
            'kp': kp, 'ki': ki, 'kd': kd, 'ku': ku, 'tu': tu, 'a': a
        }


class JoyToMavlink(Node):
    def __init__(self):
        super().__init__('joy_to_mavlink')

        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('mavros_udp_port', 14550)
        self.declare_parameter('depth_pid_kp', 1.0)
        self.declare_parameter('depth_pid_ki', 0.1)
        self.declare_parameter('depth_pid_kd', 0.5)

        rate = self.get_parameter('publish_rate_hz').value
        serial_port = self.get_parameter('serial_port').value
        serial_baud = self.get_parameter('serial_baud').value
        mavros_port = self.get_parameter('mavros_udp_port').value
        kp = self.get_parameter('depth_pid_kp').value
        ki = self.get_parameter('depth_pid_ki').value
        kd = self.get_parameter('depth_pid_kd').value

        self.depth_pid = PIDController(kp=kp, ki=ki, kd=kd)
        self.autotuner = RelayAutoTuner(relay_amplitude=0.4, hysteresis=0.05,
                                        min_cycles=4, timeout=60.0)

        # Direct serial connection to FC
        self.mav = None
        self.mav_lock = threading.Lock()
        try:
            self.mav = mavutil.mavlink_connection(serial_port, baud=serial_baud)
            self.mav.wait_heartbeat(timeout=10)
            self.get_logger().info(
                f'FC connected on {serial_port}: system {self.mav.target_system}')
        except Exception as e:
            self.get_logger().error(f'FC serial failed: {e}')

        # UDP socket to forward telemetry to MAVROS
        self.mavros_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mavros_addr = ('127.0.0.1', mavros_port)

        # Start serial reader thread (reads FC, forwards to MAVROS UDP)
        self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.reader_thread.start()

        # Start heartbeat thread
        self.hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.hb_thread.start()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, depth=10)

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.vfr_sub = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.vfr_cb, sensor_qos)
        self.autotune_sub = self.create_subscription(
            Bool, '/rov/autotune_trigger', self.autotune_cb, 10)

        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/mavros/override', 10)
        self.depth_sp_pub = self.create_publisher(Float64, '/rov/depth_setpoint', 10)
        self.depth_cur_pub = self.create_publisher(Float64, '/rov/depth_current', 10)
        self.dh_pub = self.create_publisher(Bool, '/rov/depth_hold_active', 10)
        self.pid_pub = self.create_publisher(String, '/rov/pid_status', 10)
        self.heartbeat_pub = self.create_publisher(Bool, '/rov/fc_heartbeat', 10)

        self.arm_client = self.create_client(SetBool, '/rov/arm')
        self.capture_client = self.create_client(Trigger, '/photogrammetry/capture')

        self.pub_timer = self.create_timer(1.0 / rate, self.publish)
        self.log_timer = self.create_timer(0.5, self.log)
        self.status_timer = self.create_timer(0.25, self.pub_status)
        self.hb_check_timer = self.create_timer(1.0, self.check_hb)

        self.surge = 0.0; self.sway = 0.0; self.heave = 0.0; self.yaw = 0.0
        self.prev_surge = 0.0; self.prev_sway = 0.0
        self.prev_heave = 0.0; self.prev_yaw = 0.0

        self.fc_armed = False; self.want_armed = False
        self.arm_btn_prev = False; self.arm_pending = False
        self.cap_btn_prev = False; self.fc_hb_ok = False

        self.depth_hold = False
        self.dpad_left_prev = False; self.dpad_up_prev = False; self.dpad_down_prev = False
        self.current_depth = 0.0; self.depth_valid = False; self.sp_step = 0.25

        self.joy_received = False; self.last_joy = self.get_clock().now()

        self.get_logger().info(f'Joy→MAVLink ready (direct serial, {rate}Hz)')

    def _serial_reader(self):
        """Background: read raw bytes from FC serial, forward to MAVROS UDP.

        Waits on the port with select() instead of polling: pymavlink opens the port non-blocking,
        so a plain read() loop spun one CPU core at 100% (with or without a Pixhawk attached)."""
        import select
        import time
        while True:
            port = self.mav.port if self.mav else None
            if port is None:
                time.sleep(0.5)          # no FC connected (joy_to_mavlink only connects at startup)
                continue
            try:
                ready, _, _ = select.select([port.fileno()], [], [], 0.2)
                if not ready:
                    continue
                data = port.read(max(1, getattr(port, 'in_waiting', 0) or 512))
                if data:
                    self.mavros_sock.sendto(data, self.mavros_addr)
                else:
                    time.sleep(0.01)     # readable but empty: device going away; don't spin
            except Exception:
                time.sleep(0.1)

    def _heartbeat_loop(self):
        """Send GCS heartbeat to keep FC alive."""
        import time
        while True:
            with self.mav_lock:
                if self.mav:
                    try:
                        self.mav.mav.heartbeat_send(6, 8, 0, 0, 0)
                    except: pass
            time.sleep(1)

    def check_hb(self):
        msg = Bool(); msg.data = self.fc_hb_ok
        self.heartbeat_pub.publish(msg); self.fc_hb_ok = False

    def state_cb(self, msg):
        self.fc_hb_ok = msg.connected
        prev = self.fc_armed; self.fc_armed = msg.armed
        if self.fc_armed != prev:
            self.get_logger().info(f'FC {"ARMED" if self.fc_armed else "DISARMED"}')
            self.want_armed = self.fc_armed
            if not self.fc_armed:
                self.depth_hold = False
                self.depth_pid.reset()
                if self.autotuner.active:
                    self.autotuner.stop()

    def vfr_cb(self, msg):
        self.current_depth = -msg.altitude; self.depth_valid = True
        m = Float64(); m.data = self.current_depth; self.depth_cur_pub.publish(m)

    def autotune_cb(self, msg):
        """Trigger auto-tune when dashboard sends True on /rov/autotune_trigger."""
        if not msg.data:
            return
        if not self.fc_armed:
            self.get_logger().warn('Auto-tune ignored: FC not armed')
            return
        if not self.depth_valid:
            self.get_logger().warn('Auto-tune ignored: no depth reading')
            return
        if self.autotuner.active:
            self.get_logger().warn('Auto-tune already running')
            return
        # Force depth hold ON, capture current depth as setpoint
        self.depth_hold = True
        self.depth_pid.setpoint = self.current_depth
        self.depth_pid.reset()
        now = self.get_clock().now().nanoseconds / 1e9
        self.autotuner.start(self.current_depth, now)
        self.get_logger().info(
            f'Auto-tune STARTED at depth {self.current_depth:.2f}m '
            f'(relay=±{self.autotuner.d}, hyst={self.autotuner.h}, '
            f'min_cycles={self.autotuner.min_cycles})')

    def _filter(self, new, prev):
        if abs(new - prev) < NOISE_THRESHOLD and abs(new) < STICK_DEADZONE: return prev
        return new

    def joy_cb(self, msg):
        if len(msg.axes) < 8 or len(msg.buttons) < 10: return
        self.last_joy = self.get_clock().now()
        if not self.joy_received:
            self.joy_received = True; self.get_logger().info('Receiving joystick data')

        self.surge = self._filter(apply_deadzone(msg.axes[1]), self.prev_surge); self.prev_surge = self.surge
        self.sway = self._filter(apply_deadzone(msg.axes[0]), self.prev_sway); self.prev_sway = self.sway
        self.yaw = self._filter(apply_deadzone(msg.axes[3]), self.prev_yaw); self.prev_yaw = self.yaw

        dl = msg.buttons[3] == 1  # Square button
        if dl and not self.dpad_left_prev: self._toggle_dh()
        self.dpad_left_prev = dl
        du = msg.buttons[4] == 1; dd = msg.buttons[5] == 1  # L1=up R1=down
        if du and not self.dpad_up_prev and self.depth_hold: self.depth_pid.setpoint -= self.sp_step
        if dd and not self.dpad_down_prev and self.depth_hold: self.depth_pid.setpoint += self.sp_step
        self.dpad_up_prev = du; self.dpad_down_prev = dd

        now = self.get_clock().now().nanoseconds / 1e9
        if self.autotuner.active and self.depth_valid:
            # Auto-tuner overrides PID with relay
            relay_out, done, gains = self.autotuner.step(self.current_depth, now)
            self.heave = relay_out
            if done:
                if gains:
                    self.depth_pid.set_gains(gains['kp'], gains['ki'], gains['kd'])
                    self.depth_pid.reset()
                    self.get_logger().info(
                        f'Auto-tune DONE: Ku={gains["ku"]:.3f} Tu={gains["tu"]:.2f}s '
                        f'a={gains["a"]:.3f}m -> kp={gains["kp"]:.3f} '
                        f'ki={gains["ki"]:.3f} kd={gains["kd"]:.3f}')
                else:
                    self.get_logger().warn('Auto-tune timed out or failed')
        elif self.depth_hold and self.depth_valid:
            self.heave = self.depth_pid.compute(self.current_depth, now)
        else:
            self.heave = self._filter(apply_deadzone(msg.axes[4]), self.prev_heave)
        self.prev_heave = self.heave

        opts = msg.buttons[9] == 1
        if opts and not self.arm_btn_prev and not self.arm_pending:
            self.want_armed = not self.want_armed; self.arm_pending = True; self._arm(self.want_armed)
        self.arm_btn_prev = opts

        tri = msg.buttons[1] == 1  # Circle button
        if tri and not self.cap_btn_prev: self._capture()
        self.cap_btn_prev = tri

    def _toggle_dh(self):
        self.depth_hold = not self.depth_hold
        if self.depth_hold and self.depth_valid:
            self.depth_pid.setpoint = self.current_depth; self.depth_pid.reset()
            self.get_logger().info(f'Depth hold ON at {self.current_depth:.2f}m')
        elif self.depth_hold: self.depth_hold = False
        else:
            self.depth_pid.reset()
            if self.autotuner.active:
                self.autotuner.stop()
                self.get_logger().info('Auto-tune cancelled by depth hold toggle')
            self.get_logger().info('Depth hold OFF')

    def publish(self):
        elapsed = (self.get_clock().now() - self.last_joy).nanoseconds / 1e9
        if elapsed > 1.0:
            self.surge = 0.0; self.sway = 0.0; self.yaw = 0.0
            if not self.depth_hold: self.heave = 0.0

        mc_x = int(self.surge * 1000)
        mc_y = int(self.sway * 1000)
        mc_z = int(500 + self.heave * 500)
        mc_z = max(0, min(1000, mc_z))
        mc_r = int(self.yaw * 1000)

        # Send MANUAL_CONTROL directly to FC serial
        with self.mav_lock:
            if self.mav:
                try:
                    self.mav.mav.manual_control_send(
                        self.mav.target_system, mc_x, mc_y, mc_z, mc_r, 0)
                except Exception as e:
                    self.get_logger().error(f'Send err: {e}')

        # Publish for dashboard display
        rc = OverrideRCIn()
        pwm_x = int(1500 + mc_x * 0.4)
        pwm_y = int(1500 + mc_y * 0.4)
        pwm_z = int(1500 + (mc_z - 500) * 0.8)
        pwm_r = int(1500 + mc_r * 0.4)
        rc.channels = [pwm_x, pwm_y, pwm_z, pwm_r, CHAN_NOCHANGE, CHAN_NOCHANGE,
                       CHAN_NOCHANGE, CHAN_NOCHANGE] + [CHAN_NOCHANGE] * 10
        self.rc_pub.publish(rc)

    def pub_status(self):
        dh = Bool(); dh.data = self.depth_hold; self.dh_pub.publish(dh)
        if self.depth_hold:
            sp = Float64(); sp.data = self.depth_pid.setpoint; self.depth_sp_pub.publish(sp)
        st = String()
        if self.autotuner.active:
            cycles = max(0, (len(self.autotuner.switch_times) - 1) // 2)
            st.data = (f'TUNING ({cycles}/{self.autotuner.min_cycles} cycles) '
                      f'sp={self.autotuner.setpoint:.2f}m')
        elif self.depth_hold:
            st.data = (f'HOLD {self.depth_pid.setpoint:.2f}m | P={self.depth_pid.kp:.2f} '
                      f'I={self.depth_pid.ki:.2f} D={self.depth_pid.kd:.2f}')
        else:
            st.data = (f'MANUAL | P={self.depth_pid.kp:.2f} '
                      f'I={self.depth_pid.ki:.2f} D={self.depth_pid.kd:.2f}')
        self.pid_pub.publish(st)

    def _arm(self, arm):
        # Arm directly via serial
        with self.mav_lock:
            if self.mav:
                try:
                    if arm:
                        self.mav.arducopter_arm()
                        self.get_logger().info('Arming via serial...')
                    else:
                        self.mav.arducopter_disarm()
                        self.get_logger().info('Disarming via serial...')
                except Exception as e:
                    self.get_logger().error(f'Arm err: {e}')
        self.arm_pending = False

    def log(self):
        a = 'ARMED' if self.fc_armed else 'DISARMED'
        hb = 'HB' if self.fc_hb_ok else 'NO-HB'
        dh = f' DH={self.depth_pid.setpoint:.2f}m' if self.depth_hold else ''
        d = f' d={self.current_depth:.2f}m' if self.depth_valid else ''
        self.get_logger().info(
            f'[{a} {hb}{dh}{d}] surge={self.surge:+.2f} sway={self.sway:+.2f} '
            f'heave={self.heave:+.2f} yaw={self.yaw:+.2f}')

    def _capture(self):
        if not self.capture_client.service_is_ready(): return
        self.capture_client.call_async(Trigger.Request())
        self.get_logger().info('Capture triggered')


def main(args=None):
    rclpy.init(args=args)
    node = JoyToMavlink()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()